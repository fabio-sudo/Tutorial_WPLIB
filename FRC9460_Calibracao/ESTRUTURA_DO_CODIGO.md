# Estrutura do código — FRC9460

Guia para achar as coisas e para escrever código novo do mesmo jeito.
Todo mecanismo segue o mesmo padrão; o telescópio 9458 foi o modelo
(ciclo em passos numerados, modelo em paralelo, diagnóstico e alertas).

## 1. Onde fica cada coisa

| Pasta (`src/main/java/frc/robot/`) | O que tem |
|---|---|
| `Robot.java`, `RobotContainer.java` | Liga o AdvantageKit, cria os subsistemas, os botões e a simulação das bolas. Também serve os layouts do Elastic na porta 5800. |
| `CanIds.java` | **Mapa único de IDs CAN.** Nenhum subsistema escreve número solto. ID ainda não escolhido = `CanIds.NOT_DEFINED` (o motor nem é criado). |
| `Constants.java` | Só o que é do robô inteiro: as portas dos controles (`OperatorConstants`). As anotações das aulas ficam no fim do arquivo. |
| `MotorLimits.java` | Faixas de corrente recomendadas pela CTRE e pela REV, com as fontes. O `HardwareMapTest` confere todos os mecanismos. |
| `RobotComponentsVisualizer.java` | Peças móveis do modelo 3D do robô (intake e torre do shooter) no AdvantageScope. |
| `util/` | Peças comuns: `ParallelModel` + `MechanismModel` (real x simulado), `MechanismDiagnostics` (execução, motor respondendo, alerta), `PhoenixUtil` (tentativas e mensagens), `SparkUtil` (grava a configuração e avisa reinício), `TalonZeroMark` (zero do pivô e do intake que sobrevive ao deploy; no Tuner X o zero aparece como 100) e `ArmModel` (braço com gravidade sem criar objetos, usado no modelo em paralelo do intake). |

**Memória no roboRIO:** o que roda a cada ciclo no robô real não deve criar objetos à toa; o coletor de lixo do Java pausa o programa inteiro (laço estourado no teste de 29/10/2026). O modelo do intake com `SingleJointedArmSim` criava ~527 KB por ciclo; com `ArmModel`, 0 (`ArmModelTest` confere). Nomes de campos do log montados uma vez (ver `Vision.key`).
| `subsystems/<mecanismo>/` | Um mecanismo por pasta: `shooter`, `centrifuga`, `intake`, `roller`, `telescopio` (9460), `telescopio_9458`, `drive`, `vision`, `fuel`. **As constantes de cada um ficam na própria pasta** (`XxxConstants`). |
| `commands/<mecanismo>/` | Comandos com mais de um passo, que usam mais de um subsistema ou têm máquina de estados: `shooter/`, `intake/`, `telescopio/`. |
| `calibration/` | SysId e rotinas de calibração do swerve e do telescópio 9458. |

Fora do código Java:

| Pasta | O que tem |
|---|---|
| `config/advantagescope/` | Layouts, modelos 3D, scripts que geram os layouts e os guias (`docs/`). Comece pelo [LEIA-ME](config/advantagescope/LEIA-ME.md). |
| `config/elastic/` | Gerador dos painéis do Elastic e os painéis prontos, em `layouts/`. O deploy copia os painéis para o robô. Veja o [LEIA-ME](config/elastic/LEIA-ME.md). |
| `logs/` | Logs da simulação (`simulacao/`) e replays (`replay/`, gerados pelo `salvar_replay.bat`, na raiz), fora do Git. |
| `src/main/deploy/` | Vai para o roboRIO: bolas da largada (`simulacao/`). |

## 2. Arquivos de um mecanismo

Exemplo com a centrífuga (os outros são iguais):

| Arquivo | Papel |
|---|---|
| `CentrifugaConstants` | Todos os números: ID (vindo de `CanIds`), ganhos, limites de corrente, física da simulação, tolerância do modelo em paralelo. |
| `CentrifugaIO` | Contrato com o motor: `@AutoLog` das entradas + comandos (`setVelocityRpm`, `stop`). |
| `CentrifugaIOTalonFX` | Motor real (Phoenix 6). Só configura, lê sinais e envia o pedido. |
| `CentrifugaIOSim` | Física (WPILib) + emulação do controle do motor. **Também é o modelo em paralelo** (`MechanismModel`). |
| `Centrifuga` | Subsistema: segurança, metas para o controle e telemetria. Não sabe se o motor é real ou simulado. |

Onde fica cada grupo de constantes:

| Pasta | Arquivo | O que tem |
|---|---|---|
| `drive/` | `DriveConstants` | Geometria, reduções, offsets dos CANcoders e sentidos de cada módulo, limites de velocidade, PID do módulo simulado |
| `vision/` | `LimelightConstants` | Limelights físicas: nome, IDs de tags aceitos, montagem no robô |
| `vision/` | `VisionConstants` | Aceitação e confiança: idade da captura, giro, distância, fila, peso X/Y |
| `vision/` | `VisionFieldConstants`, `VisionSimulationConstants` | Mapa do campo; óptica das câmeras virtuais |
| `shooter/`, `centrifuga/`, `intake/`, `roller/` | `XxxConstants` | Motor, SEGURANÇA, CONTROLE, simulação e modelo em paralelo |
| `telescopio/`, `telescopio_9458/` | `TelescopioConstants`, `TelescopioConstants_9458` | Idem, para cada telescópio |
| `fuel/` | `FuelConstants` | Física da bola, coleta, depósito, detecção e limites da constância dos tiros |
| `fuel/` | `RollingStats`, `ShotConsistency` | Estatística dos últimos 10 disparos (média, desvio, taxa de acerto) e diagnóstico em texto; sem robô, igual no replay |
| `vision/` | `PoseInitializer` | Decide quando a pose é definida pelas tags (MegaTag1): só desabilitado e parado, 2+ tags, quadros estáveis, desalinhada |
| `calibration/` | `CalibrationConstants` | Rotinas de calibração |
| raiz | `CanIds`, `MotorLimits`, `LoggingConstants`, `Constants` | O que é do robô inteiro: IDs CAN, faixas de corrente, gravação/replay, portas dos controles |

Os arquivos de constantes guardam **só valores**. Um valor derivado em uma
expressão é aceito (ex.: `132 / curso medido`, `DCMotor.getNeoVortex(1)`).
Método com `if`, validação ou leitura de arquivo/ambiente vai para outra classe:
a escolha do modo e dos arquivos de log está em `LoggingSetup`; o centro do hub
pelas tags, em `vision/FieldTags`.

`createIO()` sempre na mesma ordem: **replay → IO vazio**, **robô real →
hardware**, **computador → simulação**. O replay vem primeiro porque também roda
no computador.

## 3. O ciclo de cada subsistema (20 ms)

```java
@Override
public void periodic() {
    readSensors();        // 1. io.updateInputs + Logger.processInputs
    stepParallelModel();  // 2. a física recebe a tensão medida (real x simulado)
    applySafety();        // 3. SEGURANÇA: desabilitado, zero perdido, fim de curso
    logTelemetry();       // 4. saídas com os nomes que os layouts usam
}
```

Cada passo é um método com a sua `// #region` numerada. Os comandos rodam
depois do `periodic` e só pedem metas (posição, RPM, saída).

O telescópio 9460 calcula o PID no roboRIO, então o ciclo dele tem mais passos
(como o 9458): 1. sensores, 2. modelo, 3. segurança: travamento, 4. segurança:
o motor pode andar?, 5. controle: PID + feedforward, 6. segurança: tensão
máxima e batentes, 7. acionamento, 8. telemetria.

## 3b. Segurança × controle (PID)

Todo arquivo separa as duas coisas em regiões com o mesmo prefixo:

- `// #region SEGURANÇA - ...`: tudo que **impede ou limita** o motor
  (desabilitado, zero perdido, fim de curso, soft limits, limite de corrente,
  tensão máxima, travamento, paradas).
- `// #region CONTROLE - ...`: tudo que **calcula a meta ou a tensão**
  (ganhos kP/kI/kD/kS/kV/kG, perfil do Motion Magic, feedforward, malha aberta).

A regra de ouro: **a segurança decide primeiro, o controle calcula depois, e a
segurança limita o resultado**. No código isso aparece como
`applySoftLimits(controllerVolts())` nas simulações e como os passos 4 → 5 → 6
do telescópio.

| Mecanismo | Onde o PID roda | CONTROLE (ganhos e cálculo) | SEGURANÇA |
|---|---|---|---|
| Pivô do shooter | TalonFX (Motion Magic) | `ShooterConstants` PIVÔ - CONTROLE; `ShooterPivotIOTalonFX.createConfiguration` | Soft limits ±130° = ±8,84 voltas do rotor, corrente 20/40 A, zero perdido para o pivô |
| Top e tower | Spark Flex (velocidade) | `ShooterConstants` RODAS - CONTROLE; `ShooterWheelIOSparkFlex.createConfig` | Corrente 60 A por motor; tower para freando |
| Centrífuga | TalonFX (VelocityVoltage) | `CentrifugaConstants` CONTROLE; `CentrifugaIOTalonFX.createConfiguration` | Corrente 40/60 A; para freando (alvo 0 RPM) |
| Intake | TalonFX (PID de segurar + kG) | `IntakeConstants` CONTROLE; `IntakeIOTalonFX.createLeaderConfiguration` | Soft limits aberto/fechado, fim de curso, corrente 40 A, zero perdido |
| Rolete | — (malha aberta 30%) | `RollerConstants` CONTROLE - MALHA ABERTA | Corrente 40 A |
| Telescópio 9460 | roboRIO (perfil + PID + kG/kV/kA) | `TelescopioConstants` CONTROLE; `Telescopio` passo 5 | Tensão máxima 6 V, batentes, travamento, corrente 40/30 A, soft limits |

Nas simulações (`XIOSim`) a mesma divisão: `CONTROLE - EMULAÇÃO DO ...`
(o que o controlador faz por dentro) e `SEGURANÇA - SOFT LIMITS / LIMITE DE
CORRENTE` (o que ele corta). O `MotorConfigurationTest` confere que a
configuração de cada TalonFX aplica os limites e os ganhos das constantes.

A reorganização foi conferida sem mudar comportamento: as simulações e os
subsistemas geraram exatamente os mesmos números antes e depois (gabarito de
1710 linhas), o log do telescópio 9460 ficou idêntico ciclo a ciclo e as
configurações dos motores ficaram iguais às antigas.

## 3c. Ícones nos comentários

Para achar um assunto: `Ctrl+Shift+F` com o ícone lista todos os lugares do
projeto; `Ctrl+F` faz o mesmo dentro do arquivo aberto.

| Ícone | Assunto | O que marca |
|---|---|---|
| 🛑 | SEGURANÇA | Tudo que impede ou limita o motor: parada, trava, zero perdido, desabilitado |
| ⚡ | CORRENTE | Limites de corrente e de tensão |
| 🎯 | PID | Ganhos kP, kI, kD, kS, kV, kG, kA e onde o PID roda |
| 🚀 | VELOCIDADE | Velocidade máxima, aceleração, RPM, perfil e saída em % |
| 📏 | MEDIDAS | Curso, reduções, geometria e offsets |
| 🧭 | POSE | Zero, heading, pose e referência do piloto |
| 🎮 | BOTÕES | O que cada botão faz. O mapa completo está em `RobotContainer.configureBindings` |
| 📷 | VISÃO | Limelight, filtros e peso das medições |
| 🔁 | CICLO | A ordem do que roda a cada 20 ms |
| 🧪 | SIMULAÇÃO | Física e modelo em paralelo |
| 📊 | TELEMETRIA | Log, diagnóstico e alertas |
| 🔧 | CALIBRAR | Rotinas e valores para medir no robô |
| 🚧 | ATENÇÃO | Ponto do pente fino de 31/10/2026 que merece revisão |
| 📦 | ARQUIVADO | Código antigo guardado só para consulta |

Regras:

- O ícone vem logo depois de `#region`: `// #region 🛑⚡ SEGURANÇA - LIMITES DE CORRENTE`.
  Dois ícones juntos valem para os dois assuntos.
- Os prefixos `SEGURANÇA -` e `CONTROLE -` continuam: procurar pela palavra também funciona.
- `🚧` sai do código quando o ponto for resolvido.
- Nos arquivos `XxxIOTalonFX`, comentário não pode ter o formato de uma leitura de
  sinal (`nome = motor.getAlgo();`): o `CanSignalRatesTest` lê o arquivo como texto.

## 4. Modelo em paralelo (real x simulado)

Ideia do telescópio 9458, agora em todos os mecanismos (`util/ParallelModel`):
a cada ciclo, o XIOSim recebe **a tensão que o motor aplicou de verdade** e
prevê posição, velocidade e corrente. O log mostra os dois lado a lado.

| Situação | O que o modelo faz |
|---|---|
| Desabilitado ou leitura inválida | Realinha com o real (copia posição e velocidade). |
| Habilitado | Anda sozinho, só com a tensão medida. |
| Pivô, intake e telescópio: real **e** modelo parados | Guarda o erro (`DiferencaAoParar`) e realinha. Cada movimento é comparado desde o início. |
| Replay | Não roda (o log original já tem as saídas do modelo). |

Como ler:

- **diferença pequena**: o modelo está bom e a simulação vale para treinar;
- **diferença que cresce devagar**: atrito, inércia ou redução diferentes do modelo;
- **salto**: batida, travamento, bola presa ou zero manual errado;
- **`Divergindo`**: passou da tolerância por mais de 0,5 s (`Diagnostico/Resultado = REAL_DIFERENTE_DO_MODELO`).

Na simulação o "real" é o próprio XIOSim, com efeitos que o modelo não tem. A
diferença mostra exatamente esses efeitos: cada bola no rolete (corrente e RPM),
a energia que cada bola tira do top e o erro do zero manual no pivô.

| Mecanismo | Chave no log | Grandeza | Tolerância (calibrar no robô) |
|---|---|---|---|
| Pivô do shooter | `Shooter/Pivo` | graus | 3° |
| Top | `Shooter/Top` | RPM | 500 (o tiro contínuo deixa ~300 abaixo) |
| Tower | `Shooter/Tower` | RPM | 300 |
| Centrífuga | `Centrifuga` | RPM | 300 |
| Intake | `Intake` | graus (0 = aberto) | 5° |
| Rolete | `Roller` | RPM | 500 (cada bola tira até ~600 por um instante) |
| Telescópio 9460 | `Telescopio` | cm | 3 cm |

Chaves: `<mecanismo>/Modelo/*` (válido, comparando, tensão usada) e
`<mecanismo>/Comparacao/Real*`, `Simulado*`, `Diferenca*` (real − simulado),
`DiferencaMaxima*`, `Tolerancia*`, `Divergindo`, `DiferencaAoParar*`.

Detalhes que já foram medidos:

- rodas e pivô usam um passo de 20 ms (física linear, exato e leve no roboRIO);
- o braço do intake usa subpassos de 1 ms: com um passo só, a física não linear
  errava ~0,3° por ciclo;
- as simulações aplicam o limite de corrente dos controladores (TalonFX 60 A na
  centrífuga, Spark Flex 60 A por motor). Antes a partida chegava a 300 A.

## 5. Diagnóstico e alertas

`util/MechanismDiagnostics` grava, para cada mecanismo:

- `Diagnostico/Execucao`: `REPLAY`, `ROBO_REAL_TALONFX_ID_36`, `SIMULACAO_FISICA`...
- `Diagnostico/MotorRespondendo` e um **alerta de erro** da WPILib quando o motor
  some do CAN (aparece no Driver Station e no widget Alerts do Elastic);
- `Diagnostico/Resultado`: `OK`, `MOTOR_SEM_RESPOSTA`, `ZERO_PERDIDO` ou
  `REAL_DIFERENTE_DO_MODELO`. Este último fica só no log e no Elastic: os modelos
  ainda não foram calibrados no robô real;
- `<mecanismo>/Comando`: o comando que está usando o subsistema.

O swerve usa o mesmo diagnóstico, um por módulo (`Drive/Module0` a `Drive/Module3`). O
`Resultado` diz o que procurar no robô: `DRIVE_SEM_RESPOSTA`, `STEER_SEM_RESPOSTA` ou
`CANCODER_SEM_RESPOSTA`. O código não para o módulo: só avisa.

O intake também confere o seguidor: `SEGUIDOR_SEM_RESPOSTA`. Aqui também é só um
aviso, e o líder continua empurrando sozinho.

`Shooter/Pivo/ZeroResultado` e `Intake/ZeroResultado` nunca dizem "zerado" com o zero
inválido: mostram `ZERO_PERDIDO`, que cobre falha ao zerar, motor reiniciado e posição
desconhecida ao iniciar.

O telescópio 9460 está desligado em `CanIds` (`ELEVATORS_HARDWARE_ENABLED`): no
robô real ele mostra um **aviso** (não erro), o motor não é criado e a telemetria
dele não é gravada. Na simulação e no replay ele continua completo.

## 5b. Tráfego do CAN

Os IOs reais do Phoenix chamam `PhoenixUtil.applySignalRates`, que faz duas coisas
nesta ordem: pede a frequência de cada sinal lido e só então desacelera o resto
para 4 Hz (`optimizeBusUtilization`). Se a primeira etapa falhar, a segunda não roda.

- **Sinal novo:** incluir na lista de `setUpdateFrequencyForAll` do IO. Esquecido,
  ele fica a 4 Hz sem dar erro. O `CanSignalRatesTest` reprova.
- **Seguidor:** o líder precisa manter o sinal que o seguidor copia, a 20 Hz ou
  mais: `MotorVoltage` no `PositionVoltage` e `DutyCycle` no `DutyCycleOut`.
- **CANcoder:** fica de fora. O steer usa `FusedCANcoder` e recebe a posição dele
  pelo CAN.
- **Falha:** não para o mecanismo. Aparece o aviso `[FRC9460] CAN:` na Driver
  Station e o dispositivo segue com o tráfego padrão.
- **Emergência:** `PhoenixUtil.OPTIMIZE_CAN_BUS = false` e deploy devolvem o padrão
  sem desligar o robô. Só apagar a chamada não basta: a otimização fica gravada no
  dispositivo até ele reiniciar.

Os Spark Flex (top, tower e rolete) não passam por aqui: as frequências deles são
configuradas pelo REVLib e continuam no padrão. A configuração é gravada no próprio
Spark ao iniciar (`util/SparkUtil`). Se a gravação falhar, só há um aviso. Um reinício
aparece como `[FRC9460] Spark Flex N reiniciou` na Driver Station.

## 6. Comandos

- Ficam em `commands/<mecanismo>/`, um por arquivo, com nome do que fazem:
  `ColetarCommand`, `ShooterTrackAndShootCommand`, `TelescopioAbrirCommand`...
- O subsistema oferece o controle (`setGoalMeters`, `stop`, `setRpm`...); o
  comando decide a sequência.
- Atalhos de "segurar o botão" de um passo só (`runEnd`) podem ficar no
  subsistema: `Intake.openCommand`, `Roller.pullCommand`, `Centrifuga.testCommand`.
- Testes e calibrações que movem o robô precisam de `calibracaoLiberada`
  (`DriverMechanismBindings.calibrationAllowed`): com FMS, os botões não fazem nada.
  O `DriverMechanismBindingsTest` reprova botão novo sem essa trava.

## 7. Visão no tiro

A mira usa a pose do Drive (odometria + Limelight). Em cada tiro o
`FuelTracker` grava `Fuel/Tiro/Visao/*`: se a Limelight já corrigiu a pose,
se a correção é recente (≤ 1 s, `Drive.VISION_RECENT_CORRECTION_SECONDS`), há
quanto tempo, qual câmera e, na simulação, o erro entre a pose estimada e a
verdadeira. Na simulação a bola sai da pose **verdadeira**, enquanto o código
mira pela estimada: o erro da visão desloca o tiro como no robô real.

O Drive grava a saúde da visão a cada ciclo: `Drive/Vision/CorrectedSinceReset`,
`RecentCorrection`, `CorrectionAgeSeconds`, `LastAcceptedCamera`.

## 8. Convenções de escrita

- **Regiões**: todo membro de classe dentro de `// #region NOME EM MAIÚSCULAS`
  ... `// #endregion`. Ordem usual: IMPORTS, constantes/estado, construção,
  fábrica de IO, PERIODIC e os passos numerados, SEGURANÇA, CONTROLE,
  consultas, comandos, ENCERRAMENTO (testes). Prefixos `SEGURANÇA -` e
  `CONTROLE -` sempre que o assunto for um desses (seção 3b), com o ícone
  do assunto antes do nome (seção 3c).
- **Comentários**: dizem *por que* (o que acontece se tirar), em português.
- **Nomes no log**: em português, por mecanismo (`Shooter/Pivo/AnguloGraus`).
  Drive e Vision mantêm os nomes em inglês que os layouts antigos já usam.
  **Não mude o tipo de uma chave** (ex.: `Pose3d[]` para `Translation3d[]`):
  o layout antigo para de mostrar sem avisar.
- **Limites de corrente**: sempre dentro de `MotorLimits` (o teste falha se sair).

## 9. Layouts e testes que os mantêm em dia

| AdvantageScope (`config/advantagescope/layouts/`) | Elastic (`config/elastic/layouts/`) | Gerador |
|---|---|---|
| `FRC9460-Shooter.json` | `FRC9460-Shooter.json` | `preparar_layout_shooter.py` / `preparar_layouts_elastic.py` |
| `FRC9460-CicloDaBola.json` | `FRC9460-CicloDaBola.json` | `preparar_layout_ciclo_da_bola.py` / idem |
| `Telescopio9460-*.json` | `Telescopio9460.json` | `adicionar_real_x_modelo_telescopio_9460.py` / idem |
| — | `FRC9460-Diagnostico.json` (pit) | `preparar_layouts_elastic.py` |

O `CicloDaBolaIntegrationTest` roda o robô inteiro na simulação e confere que
toda chave desses layouts existe no log e que cada widget do Elastic tem o tipo
certo. Mudou um nome no código? Rode o gerador e o teste.

Rodar (o Gradle precisa do JDK da WPILib):

```bash
export JAVA_HOME="C:/Users/Public/wpilib/2026/jdk" && ./gradlew.bat test cicloDaBolaTest shooterLogTest --offline
```

Os outros testes de integração: `visionReplayTest`, `telescopioLogTest`,
`telescopioReplayTest`, `telescopio9458LogTest`.

## 10. Mecanismo novo: lista de conferência

1. ID em `CanIds` (e na lista `ALL`), limites dentro de `MotorLimits`.
2. Pasta `subsystems/<mecanismo>/` com Constants, IO, IOReal, IOSim e o subsistema.
   As constantes dele vão no `XxxConstants` da pasta, nunca no `Constants.java` geral.
   Nas constantes, só valores: métodos e contas com `if` ficam no subsistema ou no IO.
3. O IOSim implementa `MechanismModel`; informe a tensão **média do ciclo**.
4. `periodic` em quatro passos; `ParallelModel` e `MechanismDiagnostics` no subsistema.
5. Comandos de vários passos em `commands/<mecanismo>/`.
6. Abas no layout do AdvantageScope e no Elastic; rode os geradores e o `cicloDaBolaTest`.
7. Teste com o IOSim (ex.: `IntakeTest`), limites no `HardwareMapTest` e a
   configuração do motor (`createConfiguration()`) no `MotorConfigurationTest`.
8. Regiões `SEGURANÇA -` e `CONTROLE -` separadas nas constantes, no IO real,
   no IOSim e no subsistema.
