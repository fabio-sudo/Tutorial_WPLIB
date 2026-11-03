# Shooter — simulação e AdvantageScope

**Controles atualizados em 29/10/2026:** no piloto, L2 segurado mira e B
segurado executa a sequência de tiro. R1 desce intake, L1 sobe e R2 puxa
o roller. Os toggles B/START+B descritos nas instruções históricas abaixo
foram arquivados. Consulte [Controles e calibração atual](CONTROLES_E_MECANISMOS_2026-10-29.md).

Layout: `layouts/FRC9460-Shooter.json` (**File > Import Layout...**).
Gerador: `scripts/preparar_layout_shooter.py`.
Código: `src/main/java/frc/robot/subsystems/shooter/`.

O shooter foi portado do código usado na competição (`ProjetoFrc`, agosto de
2026): mesmos IDs CAN, sentidos, limites, ganhos e curva de RPM. A diferença
é a organização: agora existe uma camada IO (robô real / simulação / replay),
como na visão, e tudo fica gravado para o AdvantageScope.

## Por que ele depende da odometria e da Limelight

O shooter mira por **coordenada de campo**: com a pose do robô, calcula a
distância e a direção do hub. A pose vem do `Drive`, que soma rodas, Pigeon
e as correções da Limelight. Pose errada = mira e RPM errados.

Por isso ele só atira depois que a Limelight corrigiu a pose ao menos uma vez
(`Shooter/Mira/PoseConfiavel`). Depois disso, mesmo sem ver tag, a odometria
continua levando a pose.

O alvo é o centro do hub da **aliança da Driver Station**, calculado pelas
tags do mapa 2026: azul (4,626; 4,035) e vermelho (11,916; 4,035).
O código antigo usava sempre o azul.

## Zero manual (sem sensor)

O pivô não tem sensor absoluto. Ao ligar o robô, o encoder do motor vale 0
onde o shooter estiver. **Centralize o shooter à mão antes de ligar.**

- Curso: ±130° = ±8,8439 voltas do rotor (14,699° por volta, medido no Tuner
  em 29/10/2026). O Talon bloqueia além disso.
- **Deploy não zera mais** (desde 29/10/2026). O Talon guarda o centro como
  **100 voltas**; reiniciar o código mantém o centro, mesmo com o shooter virado.
  Só o Talon recém-ligado (perto de 0) é zerado no início. No Tuner X o centro
  aparece como 100 e os limites como 91,156 e 108,844.
  `Shooter/Pivo/Inputs/StartupZero` mostra ZERAR_AQUI, MANTER_ZERO ou
  POSICAO_DESCONHECIDA (aí o pivô não se move até zerar).
- Zerou errado? Com o robô **desabilitado**, centralize e aperte
  **BACK + START** no controle do operador (ou `/Tuning/Shooter/ZerarPivoNoCentro`).
  Habilitado, o pedido é recusado.
- Se o Talon reiniciar (queda de bateria), o zero se perde: o pivô para e
  `Shooter/Pivo/ZeroValido` fica falso até zerar de novo.

Na simulação dá para ver o efeito de um zero mal feito: mude
`PIVOT_SIM_MANUAL_PLACEMENT_ERROR_DEGREES` em `ShooterConstants` (por exemplo
20) e veja na aba **SIM SHOOTER - Zero manual** o ângulo real batendo na fiação
antes do soft limit.

## Para que serve cada arquivo

Os comandos que usam mais de um subsistema ou têm lógica própria ficam em
`src/main/java/frc/robot/commands/`, como no ProjetoFrc:
`commands/shooter/ShooterTrackAndShootCommand` (mira + rodas + centrífuga),
`commands/shooter/ShooterAimAtHubCommand` (mira contínua) e
`commands/intake/ColetarCommand` (intake + rolete). Os comandos simples, de
um subsistema só (abrir, fechar, puxar, pivô manual, zerar), ficam no próprio
subsistema, que é o que a documentação do WPILib recomenda.

A centrífuga, o intake e o rolete têm pastas próprias; veja
[CICLO_DA_BOLA.md](CICLO_DA_BOLA.md). A sequência de tiro deste guia comanda o
shooter e a centrífuga juntos.

| Arquivo | Papel |
| --- | --- |
| `ShooterConstants` | Todos os números (IDs, curso, ganhos, curva, estimativas da simulação) |
| `ShotCalculator` | Contas puras: alvo da aliança, mira do pivô, RPM por distância, tiro andando |
| `Shooter` | Subsistema: lê os IOs, protege, grava telemetria, oferece os comandos |
| `commands/shooter/ShooterTrackAndShootCommand` | Sequência: mira → acelera → liga a centrífuga só com tudo pronto |
| `ShooterPivotIO` / `...TalonFX` / `...Sim` | Motor que gira o shooter: contrato, robô real, física |
| `ShooterWheelIO` / `...SparkFlex` / `...Sim` | Rodas por velocidade: top e tower |
| `ShooterVisualizer` | Visão de cima e linhas de mira (o giro no modelo 3D fica em `RobotComponentsVisualizer`) |
| `ShooterDashboard` | Botões em `/Tuning/Shooter`, para testar sem segundo controle |

## Testar na simulação

1. **WPILib: Simulate Robot Code** com a Simulation GUI.
2. Na Simulation GUI, em **FMS**, confira a aliança. O padrão é **Red 1**:
   o shooter mira no hub VERMELHO (x = 11,9 m), que fica na frente do robô
   quando ele começa em (2, 2) apontando para +X. Escolha **Blue 1** para o
   hub azul, ou gire o robô.
3. **Teleoperated** e **Enabled**. Dirija com o controle em **Joysticks [0]**.
4. AdvantageScope: **File > Connect to Simulator**, importe o layout.
5. **Um toque em B** liga a mira automática: o pivô segue o hub o tempo
   todo (rodas paradas). Outro toque desliga.
   **Um toque em START + B** liga o tiro automático: mira, acelera e atira
   sempre que tudo estiver pronto. Outro toque desliga e o shooter volta para
   a mira automática, se ela estiver ligada. Desabilitar o robô desliga os dois.
6. O shooter aponta para trás no zero e só alcança ±90°. Com o hub na frente,
   o pivô vai até o limite, o estado fica `HUB_FORA_DO_CURSO_GIRE_O_ROBO` e
   ele não atira: gire o robô até o hub ficar atrás.

Outras formas de acionar: controle do operador em **Joysticks [1]** (RB atira,
LB só mira) ou `/Tuning/Shooter/MirarEAtirar` no AdvantageScope.

Também há um log pronto, gravado com o robô inteiro (Drive + câmeras virtuais +
shooter): `logs/simulacao/Shooter_Demonstracao.wpilog`. Nele o robô gira, atira,
anda, o pivô acompanha o hub e chega ao limite de 90°. Em
`logs/simulacao/Shooter_BotaoB_Vermelho.wpilog` (aliança vermelha, botão B):
o pivô vai a -90° com o hub na frente, o robô gira e o tiro sai.

## Botões

| Controle | Entrada | Ação |
| --- | --- | --- |
| Piloto (porta 0) | **B (um toque)** | Liga/desliga a mira automática (pivô segue o hub) |
| Piloto (porta 0) | **START + B (um toque)** | Liga/desliga o tiro automático |
| Operador (porta 1) | RB segurado | Mirar e atirar enquanto segura |
| Operador (porta 1) | LB segurado | Só mirar (rodas paradas) |
| Operador (porta 1) | A segurado + analógico direito | Pivô manual lento (teste de sentido) |
| Operador (porta 1) | BACK + START, desabilitado | Centro atual vira o zero |

## Abas do layout

| Aba | O que mostra |
| --- | --- |
| SHOOTER 1 - Campo 3D | Robô com o shooter girando, linha laranja (para onde aponta), verde (até o alvo), pose da Limelight |
| SHOOTER 2 - Robo de perto | O mesmo, com a câmera perto do robô |
| SHOOTER 3 - Campo 2D | Visto de cima, com as duas linhas |
| SHOOTER 4 - Visto de cima | Desenho do robô: frente, limites de ±90°, shooter e direção do hub |
| SHOOTER 5 - Pivo | Ângulo, alvo, ângulo necessário, limites, tensão e corrente |
| SHOOTER 6 - Rodas (RPM) | RPM medido e alvo de top (dois motores), tower e centrífuga (`Centrifuga/...`) |
| SHOOTER 7 - Sequencia de tiro | Estado da sequência e cada condição de pronto |
| SHOOTER 8 - Mira e Limelight | Erro de direção, distância e correções aceitas da visão |
| SHOOTER 9 - Entradas | Tabela do que os IOs leram (o que o replay reproduz) |
| SHOOTER 10 - Visao: a pose da mira e confiavel? | Idade da última correção da Limelight, câmeras vendo tag, correções aceitas e erro da pose (simulação) |
| SHOOTER 11 - Pivo: real x modelo | Ângulo real × modelo em paralelo, diferença, tolerância e resultado do diagnóstico |
| SHOOTER 12 - Top: real x modelo | RPM real × modelo; no tiro contínuo o real fica abaixo (energia das bolas) |
| SHOOTER 13 - Tower: real x modelo | RPM real × modelo |
| SHOOTER 14 - Correntes: real x modelo | Corrente medida × prevista do pivô, top e tower |
| SHOOTER 15 - Diagnostico dos motores | Execução (real/simulação/replay), motor respondendo, resultado, comando e maiores diferenças |
| SIM SHOOTER - Zero manual | Ângulo do encoder × ângulo real (só simulação) |

Quando as linhas laranja e verde ficam sobrepostas, o shooter está apontado.
Se nada se mexe no 3D, confira se a fonte **Component** está logo abaixo do
**Robot** (é a lista `Mecanismos/Componentes`: model_0 = intake, model_1 = shooter).
O intake (model_0) se mexe pelo subsistema Intake ([ciclo da bola](CICLO_DA_BOLA.md)).

## Real × modelo e visão na mira

Cada motor do shooter roda um **modelo em paralelo** (padrão do telescópio
9458): a física da simulação recebe a tensão que o motor aplicou de verdade e
o log mostra lado a lado o real e o previsto (`Shooter/Pivo|Top|Tower/Comparacao/...`).
No robô real, isso mostra se a simulação está fiel; na simulação, mostra o
que o modelo não tem (a energia de cada bola no top, o zero manual errado no
pivô). Como ler cada diferença: [ESTRUTURA_DO_CODIGO.md](../../../ESTRUTURA_DO_CODIGO.md#4-modelo-em-paralelo-real-x-simulado).

A aba **SHOOTER 10** responde "a pose da mira é confiável?": se a Limelight já
corrigiu a pose, se a última correção tem menos de 1 s, qual câmera e, na
simulação, o erro da pose estimada. O painel `FRC9460-Shooter` do Elastic
mostra o mesmo ao vivo ([config/elastic](../../elastic/LEIA-ME.md)).

## Diferenças em relação ao código de competição

- O alvo segue a aliança (antes era sempre o hub azul).
- Com o hub fora do curso de ±90°, não atira (antes atirava com o pivô no limite,
  na direção errada).
- Ajustes da mira que ficavam no SmartDashboard viraram constantes: assim o
  replay repete exatamente a mesma conta.
- Perda do zero por reset do Talon agora é detectada.
- Não atira além de 5,7 m (a curva vai até 5,2 m); antes atirava com o RPM
  máximo e errava. `BLOCK_SHOTS_BEYOND_CURVE = false` volta ao antigo.
- Ao parar, a tower e a centrífuga freiam (antes ficavam soltas e ainda
  subiam bolas depois de soltar o botão).
- O "pronto por tempo" (0,75 s) foi mantido, mas fica marcado em
  `Shooter/Sequencia/ProntoPorTempo`. Para exigir o RPM, use
  `USE_SPINUP_TIME_FALLBACK = false`.

## O que conferir no robô (não dá para provar na simulação)

- **Redução do pivô:** medida no Tuner: 8,98 voltas = 132° (≈24,49:1). Conferir o centro
  depois de ir e voltar (no CSV de 29/10 a volta parou 10,7° fora).
- **Posição do shooter:** o código de competição usa 5 cm para trás e 14 cm
  para a **direita**; o CAD mede 14,4 cm para trás e 14,5 cm para a **esquerda**.
  Medir no robô e corrigir `SHOOTER_OFFSET_*`.
- **Sentido:** `PIVOT_DIRECTION_SIGN = 1` (o padrão do SmartDashboard antigo).
  Com o robô levantado, ângulo positivo deve girar anti-horário visto de cima.
- **Motores e kV:** o kV dos Spark Flex usa a velocidade do NEO. Se o top for
  NEO Vortex (como no CAD), a simulação (sem atrito) mostra o top uns 7% acima
  do alvo, sem entrar na tolerância: o tiro sai pelo tempo. No robô o atrito
  muda isso; olhar o RPM real na aba **SHOOTER 6** no primeiro teste.
- **Alturas:** saída a 0,61 m (CAD) e boca do hub a 1,829 m (modelo oficial do
  campo). O código antigo usava 0,80 e 2,49 m; isso não muda o RPM, só o desenho
  e a física das bolas.
- Inércias e motores da simulação são estimativas (`ShooterConstants`, região SIMULAÇÃO).

## Validação automatizada

`gradlew build` inclui:

- `ShotCalculatorTest`: curva igual à da competição, centros dos hubs, mira e tiro andando;
- `ShooterPivotIOSimTest`: Motion Magic, soft limits, batente da fiação e zero manual errado;
- `ShooterTest`: espera a Limelight, atira com o hub atrás, gira 45°, não atira fora do curso;
- `ParallelModelTest`: o modelo em paralelo alinha desabilitado, anda com a tensão
  medida, acusa travamento depois de 0,5 s e realinha com real e modelo parados;
- `shooterLogTest` (JVM separada): grava um tiro pelo painel e confere que toda
  chave do shooter usada por este layout existe no log.

Para regenerar o layout, na raiz do projeto:

```powershell
python config/advantagescope/scripts/preparar_layout_shooter.py
```
