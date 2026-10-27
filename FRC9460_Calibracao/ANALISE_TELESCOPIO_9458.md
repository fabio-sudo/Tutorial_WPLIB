# Análise do telescópio 9458 — 22/10/2026

## Atualização: chegada mais rápida após o novo CSV

O arquivo `log.csv` foi substituído por um registro novo, com 8.363 linhas. Os
resultados abaixo são desse registro; a análise anterior foi preservada como
histórico e não representa o conteúdo atual do arquivo.

- O novo registro mostra aproximadamente 0,62 s para completar os últimos 3 cm.
- Reduzida a zona de aproximação de 3 para **2 cm**, mantendo velocidade central
  de **0,55 m/s**, aproximação a **0,06 m/s**, aceleração de **1 m/s²**, ganhos
  atuais e margem superior de **5 cm**.
- Nos testes com os ganhos reais e atraso de leitura de 20 e 40 ms, a chegada
  passou de 0,56–0,66 s para **0,42–0,48 s**. Esses tempos são simulados.
- Os **121 testes passaram**, incluindo os limites de velocidade próximos aos
  extremos. Versão publicada: `APROXIMACAO_CURTA_ZERO_TOLERANTE_3`.
- Sem deploy; o resultado no mecanismo real ainda precisa ser medido.

A [CTRE usa MotionMagicVoltage no gerador de elevadores](https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-elev/tuning.html)
e recomenda calibrar com o mecanismo montado e carregado. O código atual continua
com perfil e PID no roboRIO, enviando tensão ao Talon. Uma eventual migração
exigiria converter as unidades dos ganhos e validar novamente o controle.
A faixa de 2 cm é um ajuste deste projeto, não uma recomendação numérica da CTRE.

## Atualização após análise do CSV e relato de batida ao recolher

A [análise do ensaio de 22/10](config/advantagescope/docs/telescopio_9458/ANALISE_CSV_2026-10-22.md)
documenta os dados e as limitações. O registro mostra aceleração de aproximadamente
3 m/s² e chegada inferior acima de 0,4 m/s a menos de 1 cm do zero.

- Corrigida a comparação exata da meta com zero, que falhava com resíduos de
  arredondamento e podia reenergizar a sustentação depois de recolher.
- Conforme pedido da equipe, velocidade central de 0,55 m/s. Mantidas aceleração
  de 1 m/s², aproximação dos extremos a 0,06 m/s e margem superior de 5 cm.
- Layout com dois modelos 3D, voltas após a redução, comparação físico/simulado em
  cm e m, limites e identificação da configuração publicada pelo programa.
- Verificação completa: **120 testes passaram**, incluindo aproximação com ganhos
  reais e atrasos simulados de leitura de 20 e 40 ms, e publicação dos novos campos.
- Sem deploy nesta revisão. Ganhos atuais preservados; validação física ainda pendente.

## Registro da revisão estrutural anterior

- O projeto compila. Os **24 testes do 9458** passam: 17 de controle, 6 do adaptador
  TalonFX e 1 de integração do log.
- Antes desta revisão, **3 testes falhavam**. Eles ainda esperavam a abertura até o
  batente físico (0,996 m), mas o código já parava na margem de 5 cm escolhida pela
  equipe (0,946 m). Os testes foram atualizados para a margem. O controle não mudou.
- `Telescopio_9458.periodic()` tinha cerca de 390 linhas, com controle e telemetria
  misturados. Agora ele chama seis etapas nomeadas. Os nomes publicados no log não
  mudaram, então os layouts do AdvantageScope continuam funcionando.
- Corrigido: o campo `Estado` continuava mostrando `DESABILITADO` (ou uma falha
  antiga) depois de habilitar o robô sem comando. Agora mostra `PARADO`. A recusa
  de meta (`META_RECUSADA`) continua visível. Um teste novo cobre esse caso.
- Nada foi enviado ao robô. O PID e o feedforward reais **ainda não foram calibrados**.

## Controle por Motion Magic (22/10, tarde)

O controle de posição do robô real passou do roboRIO para o **Motion Magic do TalonFX**.
O motivo está nas medições: no roboRIO, o encoder chegava com 20–40 ms de atraso, e cerca
de 2,6% das leituras vinham repetidas. Isso limitava o kP, impedia usar kD (com kP 55 /
kD 3,7 o telescópio pulou) e fazia o telescópio chegar atrasado e bater nos fins de curso.
Com o perfil trapezoidal sem jerk, acelerar mais deixava o movimento seco.

| | roboRIO (antes) | Motion Magic (agora) |
|---|---|---|
| Onde roda o PID | roboRIO, 50 Hz | Talon, 1 kHz, encoder sem atraso |
| Perfil | Trapezoidal, 0,55 m/s, 1 m/s² | Curva em S: 0,75 m/s, 2,5 m/s², jerk 25 m/s³ |
| Curso 0 → 0,946 m (simulação) | 2,54 s | 2,12 s |
| Ao soltar em velocidade máxima | ~15 cm | ~17 cm, a 0,75 m/s, sem voltar (conta 30 ms até a meta chegar ao Talon) |
| Trecho lento antes dos batentes | 2 cm a 6 cm/s | Igual (troca decidida pela referência do Talon) |
| Teto de tensão | 6 V | 9 V (o feedforward a 0,75 m/s pede 7,3 V) |
| Sinais do Talon | 50 Hz | 100 Hz |

**Como funciona:**
- O subsistema escolhe a meta e a velocidade de cruzeiro (normal ou lenta perto dos
  batentes) e calcula o ponto de parada ao soltar (`TelescopioMotionProfile_9458`).
- O Talon gera a curva em S e fecha a malha.
- As proteções continuam no subsistema e no IO: desabilitar, posição inválida, falha de
  comando, travamento, apoio sem tensão no fundo, soft limits, motor safety e idade dos sinais.
- O **Motion Magic dinâmico** exige Phoenix Pro e CANivore, e o Talon está no CAN do
  roboRIO. Por isso a troca para o trecho lento regrava a configuração do Talon, no
  máximo 2 vezes por movimento, esperando no máximo 25 ms pela confirmação. A CTRE
  recusa configuração com espera 0 (`TimeoutCannotBeZero`); foi isso que travou o
  telescópio real ("subiu e parou"). Agora uma troca recusada só fica no diagnóstico
  (`ConfigurationStatus = TROCA_DE_TRECHO_RECUSADA`) e o movimento continua.
- Ao soltar R2/L2, a meta de parada conta o atraso até chegar ao Talon (o próximo ciclo,
  mais a idade da leitura) e usa a maior distância de parada entre o perfil ótimo e o
  da média móvel. Assim a referência nunca passa da meta e volta, qualquer que seja a
  forma da curva em S do Talon. Soltou no trecho lento: continua lento até parar.
- Na simulação, o mesmo Motion Magic é emulado: um trapezoide passa por uma média móvel
  de A/J, o que dá jerk exato e para no alvo sem oscilar. Alvo perto demais para parar:
  freia com 2,5 m/s², passa e volta, como o Talon (antes saltava a velocidade e escondia
  o erro do atraso).
- A gravação da simulação de 22/10 às 14:30 (referência indo e voltando ±3–5 mm depois
  de soltar) era de um gerador anterior. Repetindo os mesmos comandos no código atual,
  a referência para sem voltar em todos os pontos de soltura testados.
- Para voltar ao controle antigo, basta `USE_TALON_MOTION_MAGIC = false`.

**Ajuste no robô, sem deploy** (robô **desabilitado**, campos no AdvantageScope):

| Campo | O que muda |
|---|---|
| `/Tuning/Telescopio9458/Perfil/VelocidadeMaxima` | Velocidade de cruzeiro (até 0,85 m/s) |
| `/Tuning/Telescopio9458/Perfil/Aceleracao` | Partida e frenagem (até 5 m/s²) |
| `/Tuning/Telescopio9458/Perfil/Jerk` | Suavidade: menor = mais macio (5–100 m/s³) |
| `/Tuning/Telescopio9458/Perfil/AplicarPerfil` | Aplica; resultado em `Telescopio9458/Perfil/Resultado` |
| `/Tuning/Telescopio9458/PID/kP`, `kD` + `AplicarGanhos` | PID do Talon (convertido para o Slot0) |

Com o PID a 1 kHz, o kP pode subir aos poucos a partir de 35 (por exemplo 50, depois
70), e o kD pode ser testado (0,5, depois 1). Se zumbir, volte ao valor anterior. Quando
achar os valores, grave em `MOTION_MAGIC_*` e `REAL_KP/KD`.

**Ainda precisa ser validado no robô:**
- O Talon aceitar a troca de cruzeiro no meio do movimento: o esperado é frear até
  6 cm/s antes do batente. Confira `Telescopio9458/Controle/TrechoLento` e a velocidade
  real ao chegar.
- A sensação de 2,5 m/s² com jerk de 25 m/s³. Se ainda estiver seco, baixe o jerk pelo
  painel; se estiver lento, suba a velocidade.

## Mapa dos arquivos

| Arquivo | Papel |
|---|---|
| [Telescopio_9458.java](src/main/java/frc/robot/subsystems/telescopio_9458/Telescopio_9458.java) | Subsistema: segurança, PID + perfil, feedforward, travamento, comandos e telemetria. |
| [TelescopioConstants_9458.java](src/main/java/frc/robot/subsystems/telescopio_9458/TelescopioConstants_9458.java) | Transmissão, geometria, limites, controle, física da simulação, ganhos, proteções e CAN. |
| [TelescopioGains_9458.java](src/main/java/frc/robot/subsystems/telescopio_9458/TelescopioGains_9458.java) | Ganhos kP…kA validados e gravados no log para o replay. |
| [TelescopioIO_9458.java](src/main/java/frc/robot/subsystems/telescopio_9458/TelescopioIO_9458.java) | Interface de sensores e motor (AdvantageKit). |
| [TelescopioIOTalonFX_9458.java](src/main/java/frc/robot/subsystems/telescopio_9458/TelescopioIOTalonFX_9458.java) | Robô real: Kraken X60 no CAN 5, zero, detecção de reset e configuração Phoenix 6 e Motion Magic. |
| [TelescopioMotionProfile_9458.java](src/main/java/frc/robot/subsystems/telescopio_9458/TelescopioMotionProfile_9458.java) | Curva em S: distância de parada (ao soltar e para o trecho lento) e emulação do Motion Magic na simulação. |
| [TelescopioIOSim_9458.java](src/main/java/frc/robot/subsystems/telescopio_9458/TelescopioIOSim_9458.java) | Física (`ElevatorSim`) com o limite de corrente do Talon. Também é o modelo em paralelo. |
| [TelescopioVisualizer_9458.java](src/main/java/frc/robot/subsystems/telescopio_9458/TelescopioVisualizer_9458.java) | Poses 3D da cascata (cada estágio anda metade da ponta) e Mechanism2d. |
| [TelescopioDashboard_9458.java](src/main/java/frc/robot/subsystems/telescopio_9458/TelescopioDashboard_9458.java) | Painel `/Tuning/Telescopio9458`: meta, ciclo automático, zero e ganhos. |

Unidade comum: **metros da ponta** (tubo 2"). A ponta anda 12,7 mm por volta do
rotor, e 78,4 voltas levam ao batente de 0,996 m.

## Ciclo de 20 ms

[Telescopio_9458.java:182](src/main/java/frc/robot/subsystems/telescopio_9458/Telescopio_9458.java:182)

```java
public void periodic() {
    readSensorsAndGains();   // 1. encoder, corrente, tensão + ganhos do replay
    stepParallelModel();     // 2. física em paralelo (real x simulado)
    boolean jamDetected = detectJam();   // 3. corrente alta e sem movimento
    runControl(jamDetected); // 4. segurança → PID + feedforward → motor
    feedParallelModel();     // 5. tensão do próximo passo do modelo (simulação)
    logTelemetry();          // 6. AdvantageScope
}
```

A etapa 5 estava escondida dentro de um bloco de log. Agora aparece na sequência.

## Trechos importantes

**1. Conversão do encoder para metros.** Todo o resto depende disso.
[TelescopioConstants_9458.java:54](src/main/java/frc/robot/subsystems/telescopio_9458/TelescopioConstants_9458.java:54)

```java
// 16:1 e coroa de 16 dentes (#25): 101,6 mm de corrente por volta da coroa;
// cascata: a ponta anda 2x → 78,74 voltas do rotor por metro da ponta.
MOTOR_ROTATIONS_PER_METER = GEAR_REDUCTION / (2 * Math.PI * SIM_EFFECTIVE_DRUM_RADIUS_METERS);
```

**2. Segurança antes do controle.** Qualquer uma destas condições corta a tensão.
[Telescopio_9458.java:267](src/main/java/frc/robot/subsystems/telescopio_9458/Telescopio_9458.java:267)

```java
if (DriverStation.isDisabled())      { stop(); status = "DESABILITADO"; }
else if (!hasValidPosition())        { stop(); status = "SEM_REFERENCIA_OU_COMUNICACAO"; }
else if (!inputs.controlRequestOK)   { stop(); status = "FALHA_COMANDO_MOTOR"; }
else if (positionControlActive)      { if (jamDetected && !jammed) holdWhereJammed(); driveToGoal(); }
else                                 { stayStopped(); }
```

**3. Lei de controle.** Os ganhos de calibração entram aqui.
[Telescopio_9458.java:315](src/main/java/frc/robot/subsystems/telescopio_9458/Telescopio_9458.java:315)

```java
double feedback = controller.calculate(inputs.extensionMeters, goalMeters);  // PID no perfil 0,5 m/s, 1 m/s²
double feedforward = gains.feedforward(setpoint.velocity, acceleration);     // kS·sinal(v) + kG + kV·v + kA·a
requestedVolts = limitAtEnds(clamp(feedback + feedforward, ±6 V), ..., gains.kG());
```

**4. Limites nas pontas.** No fundo, não empurra para baixo. Na margem superior,
só sustenta o peso com `kG`.
[Telescopio_9458.java:848](src/main/java/frc/robot/subsystems/telescopio_9458/Telescopio_9458.java:848)

**5. Soltar o gatilho.** Usa `v·|v| / 2a` para calcular onde o perfil consegue
parar e define essa altura como meta. Assim o telescópio freia sem voltar.
[Telescopio_9458.java:876](src/main/java/frc/robot/subsystems/telescopio_9458/Telescopio_9458.java:876)

**6. Referência no TalonFX.** O zero é aplicado na partida. Se o Talon reiniciar
(`hasResetOccurred`), a posição fica inválida até alguém confirmar o recolhimento
pelo painel, com o robô desabilitado.
[TelescopioIOTalonFX_9458.java:71](src/main/java/frc/robot/subsystems/telescopio_9458/TelescopioIOTalonFX_9458.java:71),
[:101](src/main/java/frc/robot/subsystems/telescopio_9458/TelescopioIOTalonFX_9458.java:101)

**7. Configuração Phoenix 6.** Modo Brake, estator a 30 A, alimentação 30→20 A
e soft limits de 0 a 75,25 voltas do rotor (4 cm antes do batente).
[TelescopioIOTalonFX_9458.java:203](src/main/java/frc/robot/subsystems/telescopio_9458/TelescopioIOTalonFX_9458.java:203)

**8. Ganhos do robô real.** Estes valores serão substituídos pelo SysId.
[TelescopioConstants_9458.java:162](src/main/java/frc/robot/subsystems/telescopio_9458/TelescopioConstants_9458.java:162)

## O que mudou nesta organização

- `Telescopio_9458`: campos agrupados (IO, ganhos, controle, proteções,
  diagnóstico e modelo) e `periodic()` dividido em métodos. A telemetria ficou em
  `logControl`, `logDiagnostics`, `logTurnsAndLimits`, `logParallelModel` e
  `logFixedParameters`. Os métodos públicos foram agrupados por assunto. Também foi
  criado `getStatus()`.
- `TelescopioConstants_9458`: as regiões foram reorganizadas.
  `MOTOR_ROTATIONS_PER_METER` estava em “Proteções”, os ganhos reais estavam em
  “Simulação” e havia um comentário antigo dizendo que kS não existe. Os valores
  são os mesmos. O `0.002` de `limitAtEnds` virou `RETRACTED_REST_TOLERANCE_METERS`.
- `TelescopioIOTalonFX_9458`: a fronteira CAN (`Hardware`/`TalonHardware`) ganhou
  uma região própria.
- `RobotContainer`: o comentário do RT agora informa 0,946 m.

## Pontos de atenção que continuam

1. **kP/kD reais em teste.** O feedforward (`REAL_KS/KG/KV`) já veio do SysId do
   robô real em 22/10. O kP/kD passou de 18/0 para 35/0 (LQR com 20–40 ms de atraso de
   leitura); 55/3,7 fez o telescópio pular. Com 35/0 o robô real ficou firme e sem
   tremor: atraso de 7–13 mm no perfil e acomodação em 0,7 s.
2. **Frenagem ao soltar e chegada nos fins de curso.** Com o perfil a 1 m/s², o
   telescópio real anda mais 12 cm (0,5 s) depois de soltar R2/L2. Ele também chega
   nos fins de curso a cerca de 0,18 m/s quando falta 1 cm. Com 3 m/s² o movimento
   ficou seco demais e passou a bater no fim, então a aceleração voltou para 1 m/s².
   Para não bater, foi criada uma zona de aproximação lenta (`profileGoal`): indo para
   o fundo ou o topo operacional, o perfil entra nos últimos 3 cm a 6 cm/s. Na simulação,
   o telescópio chega a no máximo 0,08 m/s a 1 cm do fim. Falta validar no robô.
   A frenagem ao soltar (12 cm) continua igual.
   As travas de corrente, travamento e teto de 6 V nunca atuaram nesses testes.
3. O prefixo `SIM_` em `SIM_CABLE_TO_TIP_RATIO` e `SIM_EFFECTIVE_DRUM_RADIUS_METERS`
   engana: essas constantes também convertem o encoder real. O nome foi mantido para
   não quebrar referências.
4. Algumas chaves de log repetem o mesmo valor (`Comparacao/DiferencaMetros` e
   `Modelo/DiferencaMetros`, `Comparacao/SimuladoMetros` e `Modelo/ExtensaoMetros`).
   Elas foram mantidas porque os layouts e a documentação as usam.
5. A posição continua dependendo de o robô ligar com o telescópio recolhido. Ainda
   não existe homing automático (veja o `calibrateZero` da CTRE abaixo).

## `Elevator.java` gerado pelo Phoenix Tuner X — o que aproveitar

O arquivo usa o **mesmo motor**: TalonFX 5 no barramento `rio`, com redução 16:1.
Para comparar, as unidades da CTRE foram convertidas para metros da ponta.
Uma volta da coroa equivale a 101,6 mm de corrente × 2 da cascata = **0,2032 m**.

| Item | Tuner X | 9458 atual | Decisão |
|---|---|---|---|
| Raio do tambor | 17,78 mm | coroa 16,27 mm + cascata 2× (32,34 mm efetivo) | **Não usar.** Valor genérico: erra a corrente em +10% e a ponta em −45%. |
| `kMaxHeight` | **0 m** | 0,996 m | **Bug do gerador:** a simulação e o Mechanism2d deles não se movem. |
| Setpoint `Top` | 4,8 voltas = **0,975 m** | máximo operacional 0,946 m (4,65 voltas) | **Fica dentro da margem de 5 cm.** Não usar sem rever a margem. |
| `Middle` | 1 volta = 0,203 m | — | Só um exemplo. |
| Motion Magic | 2,5 rot/s e 5 rot/s² = 0,508 m/s e 1,016 m/s² | 0,5 m/s e 1,0 m/s² | **Já coincide.** Nada a mudar. |
| kV | 1,96 V/(rot/s) = 9,65 V/(m/s) | 9,40 V/(m/s) | Os dois são **teóricos** (velocidade livre do Kraken). O SysId substitui. |
| kP | 64 V/rot = **315 V/m** | 18 V/m | **Não copiar.** Na CTRE roda a 1 kHz no Talon; aqui roda a 50 Hz no roboRIO. |
| kS / kG / kA | 0,2 V / 0,1 V / 0,098 V/(m/s²) | 0 / 0,168 V / 0,017 V/(m/s²) | Valores de exemplo. **Vêm do SysId.** |
| `GravityType` | `Elevator_Static` | kG constante no feedforward | Equivalente. |
| Estator | 40 A | 30 A (+ alimentação 30→20 A) | Manter 30 A até medir no robô. |
| Fins de curso | Habilitados (NA, pino do Talon) | Não instalados | **Aproveitar quando os sensores chegarem**, com `ReverseLimitAutosetPositionEnable` para zerar automaticamente no fundo. |
| Soft limits | Não tem | 0 a 75,25 voltas do rotor | Manter os nossos. |
| `SensorToMechanismRatio` | 16 (voltas da coroa) | 1 (rotor) e conversão no roboRIO | Manter 1. Mudar quebraria todas as conversões e os testes. |
| Controle | `MotionMagicVoltage` no Talon | `ProfiledPIDController` + `VoltageOut` | Manter o nosso por enquanto. Simulação, replay, modelo paralelo, travamento e testes dependem dele. |
| Manual | `DutyCycleOut` | Tensão | Não usar. O ciclo de trabalho varia com a bateria. |
| `calibrateZero()` | Desce a −10%, ignorando os limites, até bater; depois zera | Operador confirma o recolhimento | **A ideia mais útil.** Veja a observação abaixo. |
| `isHardStop` | < 1 rot/s e > 10 A, por 0,1 s | `detectJam()`: > 22 A, < 0,02 m/s, por 0,25 s | Já existe equivalente. |
| Simulação | `TalonFXSimState` a 2 ms, `KrakenX60Foc` | `ElevatorSim` a 20 ms, Kraken sem FOC | Manter a nossa. O modelo FOC só vale com licença Phoenix Pro. |

**Observação importante:** a cada partida, `TelescopioIOTalonFX_9458` aplica uma
`TalonFXConfiguration` **completa**. Tudo o que for gravado pelo Tuner X (Slot0,
fins de curso, Motion Magic) volta ao padrão quando o robô liga. Para manter algo,
copie a configuração para `createConfiguration()`.

**Sobre o `calibrateZero`:** para adaptá-lo, é preciso um comando de homing que
ignore o soft limit reverso. Hoje esse limite é 0, então não dá para descer se o zero
estiver errado. O comando também deve exigir algumas centenas de milissegundos de
movimento antes de aceitar o “batente”. Na partida, a corrente sobe com a velocidade
ainda zero, e o critério da CTRE poderia disparar logo no início. Fica como
melhoria opcional. Com os fins de curso instalados, o autoset do Talon é melhor.

## Preparação para o SysId

O que já ajuda: o painel aceita kP…kA ao vivo (somente desabilitado), e os ganhos
ficam gravados no log.

Antes de rodar o SysId:

1. **Registrar em metros da ponta e volts.** Uma `SysIdRoutine` da WPILib com log do
   AdvantageKit entrega kS, kG, kV e kA direto nas unidades de `REAL_K*`. O
   `SignalLogger` da CTRE gravaria em voltas do rotor (÷ 78,74 para converter).
2. **O curso é curto: 0,946 m.** Com o kV teórico (cerca de 9,4 V/(m/s)), uma rampa
   quasistática de 0,5 V/s percorre o curso em cerca de 6 s. Um degrau dinâmico de
   2 V chega a cerca de 0,2 m/s e percorre o curso em 4 a 5 s. Cada teste precisa parar
   antes de 0,9 m na subida e de 0,05 m na descida.
3. **O SysId precisa entrar no mesmo caminho seguro.** `io.setVoltage` já limita a
   6 V e respeita os soft limits. A rotina deve parar com o robô desabilitado, com
   posição inválida ou com travamento.
4. Escolher **Elevator** na análise para obter kG, com o preset de feedback
   “WPILib (2020-)” (controle no roboRIO a 20 ms) e unidade em metros.
5. Copiar os resultados para `REAL_KS/KG/KV/KA` (e kP/kD) em
   [TelescopioConstants_9458.java:162](src/main/java/frc/robot/subsystems/telescopio_9458/TelescopioConstants_9458.java:162).
   Para testar antes de gravar no código, use o painel com o robô desabilitado.

**Rotina criada:**
[Telescopio9458SysId.java](src/main/java/frc/robot/calibration/Telescopio9458SysId.java).
Ela continua no código, mas os botões R1/L1 do modo Test agora fazem passos de
10 cm (R1 sobe, L1 desce; cada aperto é mais um passo, soltou antes freia e para).
Para religar o SysId, veja o aviso no início de
[SYSID.md](config/advantagescope/docs/telescopio_9458/SYSID.md).

**Primeiro SysId no robô real (22/10):** kS ≈ 0,042 V, kG ≈ 0,201 V e
kV ≈ 9,36 V/(m/s), com R² = 0,9995. O kV confirma a transmissão do código, e o
kG é cerca de 20% maior que o teórico. Esses valores já estão em `REAL_KS/KG/KV`
(com kA = 0). kP = 35 e kD = 0 também foram gravados e ainda estão em teste no robô.

**Física da simulação corrigida (22/10):** o integrador `rkdp` da WPILib 2026 perdia
cerca de 9% de cada ciclo no `ElevatorSim` deste telescópio. Nos testes, o modelo
paralelo ficou até 16 cm atrás do real. `TelescopioIOSim_9458` agora integra de forma
exata, com a massa medida (6 kg) e atrito (0,05 V). Nos dados do robô real, o erro por
teste caiu para 3 mm em média (13 mm no máximo). A simulação agora aplica os mesmos soft limits do TalonFX
(abertura cortada em 0,956 m e fechamento cortado em 0). O passo a passo
da bancada está em
[SYSID.md](config/advantagescope/docs/telescopio_9458/SYSID.md).

---

## Histórico — revisão anterior às melhorias de 22/10

**Atualização — melhorias implementadas após autorização da equipe:**

- Referência invalidada após reset do Talon, com recuperação explícita somente
  desabilitado e mecanismo recolhido. Também permite recuperar configuração inicial falha.
- Diagnóstico de configuração, zero, leitura e comando; encoder sem limitar e alertas.
- Ganhos reais separados da física, ajuste no painel somente desabilitado,
  limite da contribuição integral e registro dos ganhos para replay.
- Dois telescópios 3D lado a lado no layout existente: físico pelo encoder e
  virtual pela tensão medida. Gráficos de posição, erro, tensão, velocidade e corrente.
- Proteção do modelo contra leituras inválidas e alinhamento quando desabilitado.

O procedimento atualizado está em `config/advantagescope/docs/telescopio_9458/ADVANTAGESCOPE.md`.
Os achados abaixo registram a revisão **anterior às alterações**. Números de linha
se referem às versões daquela época.

**Parecer inicial:** há uma boa base de desenvolvimento, com separação de responsabilidades,
controle de movimento, telemetria e testes. Ainda não há evidência suficiente para
considerá-lo validado para operação robusta no mecanismo real. Os principais pontos
pendentes são a referência de posição, a calibração física e o diagnóstico de falhas.

**Premissa operacional confirmada pela equipe:** o robô sempre inicia com o
telescópio fisicamente na posição zero; sensores de fim de curso serão instalados
em uma etapa posterior. Dentro desse procedimento, zerar o encoder na inicialização
é válido e não constitui, por si só, um erro do código. Essa condição também deve
valer quando o programa for reiniciado, pois o construtor executa novamente.
A ausência dos sensores planejados não é tratada como defeito da etapa atual.

Relato analisado: a simulação funciona; o motor real gira, mas a representação
virtual fica parada. Esta análise não alterou o controle nem enviou comandos ao robô.

**Atualização após confirmação do usuário:** o motor é acionado por RT/LT, com
o código no roboRIO, e o roboRIO está configurado para a equipe **9458**.
As preferências salvas do AdvantageScope foram conferidas novamente e continuam
em **10.94.60.2**. Há, portanto, uma divergência concreta entre o controlador usado
e o destino de rede salvo. Para a rede padrão da equipe 9458, o endereço é
**10.94.58.2**. Esta é a principal hipótese para a imagem não acompanhar o motor,
caso a conexão esteja usando esse endereço salvo; não foi confirmada uma conexão
ao vivo durante a análise. Uma conexão USB usa outro endereço, e um endereço
personalizado deve seguir a configuração real do roboRIO.

O projeto também mantém `teamNumber: 9460` em `.wpilib/wpilib_preferences.json`.
Isso define o destino padrão de deploy pelo GradleRIO, separadamente da conexão
do AdvantageScope. O número do projeto não foi alterado nesta revisão. Para deploy
na bancada 9458, o destino precisa corresponder à equipe 9458; não é necessário
reenviar o programa apenas para corrigir o endereço do visualizador.

### Evidências verificadas

- A compilação e os **11 testes de `Telescopio_9458Test` passaram**, sem falhas.
- O log `logs/simulacao/Simulacao_26-10-21_14-22-50.wpilog` contém 64 campos
  relacionados ao 9458. A extensão medida na simulação e a coordenada Z da ponta
  em `Componentes` variam de 0 a **0,977336 m**. O modelo paralelo também chega a
  esse valor. Portanto, nesse log, a posição chega corretamente à saída 3D.
- Os arquivos `logs/akit_26-10-21_09-48-35.wpilog` e
  `logs/akit_26-10-21_10-00-57.wpilog`, os dois mais recentes diretamente em `logs`,
  **não contêm campos `Telescopio9458`**. Eles não demonstram o funcionamento nem
  a falha do 9458 no teste relatado. Isso não prova qual versão está no roboRIO hoje.
- As preferências salvas do AdvantageScope indicam endereço **10.94.60.2** e
  modo **`nt4-akit`**. O projeto também usa equipe **9460**. Esses valores precisam
  corresponder ao roboRIO usado na bancada, que o usuário confirmou ser **9458**.
  O nome do mecanismo, por si só, não determina o número configurado no controlador.
- O estado salvo da aba 3D do AdvantageScope aponta para
  `/RealOutputs/Telescopio9458/Pose` e
  `/RealOutputs/Telescopio9458/Componentes`, com o asset `FRC9458 - Telescopio`.
  Os nomes correspondem aos publicados pelo código. Preferências e estado salvos
  não comprovam a fonte conectada nem a posição da linha do tempo durante o teste.

### Caminho da posição real até a imagem

1. `Telescopio_9458.createIO()` seleciona `TelescopioIOTalonFX_9458` no roboRIO.
2. Essa classe lê o TalonFX **CAN 5**, no barramento padrão do roboRIO.
3. A posição do motor é convertida em extensão da ponta: aproximadamente
   **78,7402 voltas por metro**, ou **12,7 mm por volta do rotor**.
4. `Telescopio_9458.periodic()` passa a extensão medida para o visualizador,
   inclusive quando o robô está desabilitado.
5. O visualizador publica duas poses: tubo intermediário em `extensão / 2`
   e tubo final em `extensão`.
6. `Robot` instala `NT4Publisher` tanto no modo real quanto na simulação.
   O AdvantageScope recebe essas saídas e anima os componentes.

A pose da base permanece na origem de propósito. Quem anima os tubos é
`Componentes`, e não `Pose`. Não encontrei um bloqueio que execute o visualizador
somente na simulação. A transmissão, a relação da cascata e os dois componentes
do asset são coerentes entre si no código analisado.

Existe também uma **previsão física paralela**, em `Telescopio9458/Modelo`.
Ela é diferente da imagem que acompanha o encoder: recebe a tensão solicitada
pelo controlador e calcula seu próprio movimento. Sua existência não significa
que o motor real esteja sendo medido corretamente.

### Achados por prioridade

**1. Premissa operacional atendida — zero automático na inicialização.**
O construtor de `TelescopioIOTalonFX_9458` grava posição zero assim que
configura o motor. O retorno bem-sucedido de `setPosition` confirma a comunicação,
enquanto a posição física é garantida pelo procedimento da equipe, agora confirmado.
Portanto, esse comportamento é coerente com a etapa atual de testes e não precisa
ser removido apenas pela ausência de sensores. Reiniciar o programa com o mecanismo
estendido violaria essa premissa e deslocaria a referência do controle e dos limites
de software. Na etapa futura, os sensores poderão confirmar a referência e permitir
um procedimento de referenciamento automático.

**2. Alta — validade da referência não acompanha reinicializações do Talon.**
*(Resolvido: `hasResetOccurred` invalida o zero.)* `zeroed` era um valor fixo
definido na construção. Não havia tratamento de reset do controlador que invalidasse
e restabelecesse a referência.

**3. Média — o controle real usa ganhos derivados da física ideal.**
*(Parcial: ganhos reais separados; a calibração continua pendente.)* O motor real
recebia o mesmo feedforward `SIM_KG`, `SIM_KV` e `SIM_KA` usado na simulação, com
massa de 5 kg e sem atrito. Caracterizar o mecanismo com sua carga real.

**4. Média — a comparação “real x modelo” misturava erro físico com diferença de tensão.**
*(Resolvido: no hardware, o modelo usa a tensão medida.)*

**5. Média — diagnóstico visual insuficiente para diferenciar zero errado de falta de dados.**
*(Resolvido: voltas sem clamp, `Diagnostico` e alertas.)* O visualizador limita
posições à faixa do desenho; a leitura sem limitação agora é publicada separadamente.

**6. Baixa — documentação e cobertura deixam lacunas de integração.**
*(Parcial: teste de integração do log criado; falta teste com o Talon físico.)*

### Como localizar a falha relatada

Usar **File > Connect to Robot > NetworkTables 4 (AdvantageKit)** no AdvantageScope,
com o endereço do roboRIO da bancada (**10.94.58.2** na rede padrão da equipe
9458), e acompanhar o instante atual. Corrigir `Robot Address` em
**App > Show Preferences** antes de reconectar. **Connect to
Simulator** conecta ao computador local (`127.0.0.1`), não ao motor no roboRIO.
Esses modos e o controle da linha do tempo são descritos na
[documentação oficial do AdvantageScope](https://docs.advantagescope.org/overview/live-sources/).

| Campo | Resultado esperado ou interpretação |
|---|---|
| `/RealOutputs/Telescopio9458/Modo` | `ROBO_REAL_TALONFX_ID_5`. Outro valor indica outra execução. |
| `/Telescopio9458/Inputs/MotorConnected` | `true` durante leitura saudável do motor configurado. |
| `/Telescopio9458/Inputs/PositionValid` | `true` indica configuração/zero aceitos pelo código; não é confirmação física de recolhimento. |
| `/Telescopio9458/Inputs/ExtensionMeters` | Deve mudar conforme o motor gira. Se não muda, investigar CAN/encoder/motor/versão do programa. |
| `/RealOutputs/Telescopio9458/Componentes` | Z do primeiro componente = metade da extensão limitada; Z do segundo = extensão limitada. |
| `/RealOutputs/Telescopio9458/Diagnostico` | `OK`; outro valor indica o primeiro motivo a investigar. |

A API Phoenix 6 disponibiliza detecção de reset por `hasResetOccurred`
([ParentDevice](https://api.ctr-electronics.com/phoenix6/stable/java/com/ctre/phoenix6/hardware/ParentDevice.html)).
`setPosition` define a leitura de posição e retorna o status da operação
([CoreTalonFX](https://api.ctr-electronics.com/phoenix6/stable/java/com/ctre/phoenix6/hardware/core/CoreTalonFX.html)).
