# Ciclo da bola — intake, roller, centrífuga, lançamento e contagem

Layout: `layouts/FRC9460-CicloDaBola.json` (**File > Import Layout...**).
Gerador: `scripts/preparar_layout_ciclo_da_bola.py`.
Log pronto para abrir: `logs/simulacao/Ciclo_Da_Bola_Demonstracao.wpilog`.

## Caminho da bola e onde está cada parte

```text
chão --(intake aberto + roller puxando)--> depósito --(centrífuga)--> tower --> top --> voo --> hub
```

| Pasta (`src/main/java/frc/robot/subsystems/`) | O que faz | Motor |
| --- | --- | --- |
| `intake/` | Abre e fecha o intake; trava a posição contra o peso | 2 TalonFX (34 líder, 35 segue invertido) |
| `roller/` | Rolete que puxa a bola (30%) | Spark Flex 31 |
| `centrifuga/` | Empurra a bola do depósito para a tower | TalonFX 36 |
| `shooter/` | Pivô, top e tower; sequência de tiro | TalonFX 32, Spark Flex 37/33/30 |
| `fuel/` | Bolas: física do voo, simulação do campo e contagem pelos sensores | — |

Cada pasta segue o padrão da visão: `...IO` (contrato), `...IOTalonFX`/`...IOSparkFlex`
(robô real), `...IOSim` (física no computador) e o subsistema. IDs, sentidos e
ganhos vieram do código de competição (`ProjetoFrc`).

O intake também começa por **zero manual**: o robô deve ser ligado com o intake
**fechado**. Fechado = 0 voltas, aberto = −19 voltas (soft limits do Talon; era −18 até o teste de 28/10/2026, quando o intake parou um pouco antes do fim).

## Botões

| Controle | Entrada | Ação |
| --- | --- | --- |
| Piloto (porta 0) | **R1 (RB) segurado** | Sobe (fecha) o intake; soltou, trava onde está |
| Piloto (porta 0) | **R2 (RT) segurado** | Desce (abre) o intake; soltou, trava onde está |
| Piloto (porta 0) | **START + R1 (um toque)** | Liga o rolete; outro toque desliga (desabilitar também desliga) |
| Piloto (porta 0) | **B (um toque)** | Liga/desliga a mira automática (pivô segue o hub) |
| Piloto (porta 0) | **START + B (um toque)** | Liga/desliga o tiro automático (a centrífuga só liga com tudo pronto) |
| Operador (porta 1) | X / Y segurado | Abre / fecha o intake |
| Operador (porta 1) | B segurado | Rolete puxando |
| Operador (porta 1) | RB segurado | Mira e atira enquanto segura |

A coleta antiga do piloto (**L3** abre e puxa, **R3** fecha) está comentada no
`RobotContainer`, com o cabeçalho "ANTIGO", para consulta.

## Testar na simulação

1. Simulate Robot Code, **Blue 1** no FMS da Simulation GUI, Teleoperated + Enabled.
2. Importe o layout e abra a aba **BOLA 1 - Campo 3D**.
3. O robô começa com **8 bolas** (pré-carga). As 456 bolas do campo vêm das
   posições oficiais de largada (`src/main/deploy/simulacao/fuel_2026_inicial.json`).
4. Dê um toque em **START + R1** (liga o rolete), segure **R2** até o intake descer
   e dirija até as bolas do centro (x ≈ 7,4 a 9,1 m): elas somem do chão quando
   entram pela boca do intake. Outro toque em START + R1 desliga o rolete.
5. Pare com o hub **atrás** do robô e dê um toque em **START + B**: as bolas
   saem em arco até cair no hub. Outro toque em START + B para de atirar.
   Linhas no 3D: **verde** = planejado, **magenta** = pelos sensores,
   **ciano** = onde chegou (simulação), **amarela** = onde a bola iria agora.

## Contagem: sensores × simulação (para comparar com o real)

| Contagem | Como é feita | Existe no robô real? |
| --- | --- | --- |
| `Fuel/Contagem/ColetasDetectadas` | Corrente do rolete acima de 12 A em duas leituras seguidas, com o rolete puxando. Só conta depois que o rolete chegou a 1800 RPM desde que foi ligado (a partida do motor parece bola) | **Sim** |
| `Fuel/Contagem/DisparosDetectados` | Queda de 100 RPM do top em 60 ms, com a centrífuga girando (≥ 1500 RPM medidos) e o top comandado | **Sim** |
| `Fuel/Contagem/NoDepositoEstimado` | Coletas − disparos detectados | **Sim** |
| `Fuel/Simulacao/Coletadas`, `Lancadas`, `Acertos`, `Erros`, `NoDeposito` | O que aconteceu de verdade na simulação | Não |

A aba **BOLA 3** mostra as duas contagens juntas. No teste automático do robô
inteiro, os sensores contaram exatamente o que a simulação fez (19 coletas e
27 disparos). **No robô real**, compare `ColetasDetectadas` e `DisparosDetectados`
com a contagem da equipe (ou vídeo) e ajuste os limites em `FuelConstants`.
As abas **BOLA 5** e **BOLA 6** desenham a corrente e a queda de RPM junto do limite.

Limitações conhecidas: bolas entrando muito juntas (menos de ~0,2 s entre elas)
mantêm a corrente alta e viram uma contagem só; e soltar o botão com uma bola
no meio do rolete pode dar uma bola a mais ou a menos. Para contar com certeza, o
caminho é um sensor de passagem (beam break) na entrada do depósito.

## Mudanças depois do log de 27/10 (simulação das 18:50)

| O que o log mostrou | O que mudou |
| --- | --- |
| Bolas saindo depois de soltar o B (pivô ainda mirando, 1,5 a 3,4 m de erro) | A centrífuga e a tower FREIAM ao parar (alvo 0 RPM) em vez de ficarem soltas |
| Tiros de 12 a 20 m com o RPM máximo | Não alimenta além de 5,7 m (curva vai até 5,2 m): estado `LONGE_DEMAIS_CHEGUE_MAIS_PERTO`. `BLOCK_SHOTS_BEYOND_CURVE = false` volta ao comportamento da competição |
| Coletas na partida do rolete sem contagem; pico falso ao soltar | O rolete só engole bola acima de 1800 RPM; parado, fica em Coast (como o Spark) |
| Disparos sem contagem depois de soltar o B | A contagem usa o RPM medido da centrífuga e exige o top comandado |
| Log de 116 MB em 8 min (trajetória prevista = 40 MB) | Trajetórias e bolas como `Translation3d`, previsão a 10 Hz com um ponto a cada 0,1 s: as chaves das bolas caíram cerca de 14 vezes |

Numa sessão de 2 minutos apertando e soltando os botões de propósito, os
sensores contaram 30 de 31 coletas e 7 de 7 disparos, com 7 acertos. O log
dessa sessão está em `logs/simulacao/Sessao_Teste_Correcoes.wpilog`.

## IDs CAN e limites de corrente

Todos os IDs ficam no mapa único `src/main/java/frc/robot/CanIds.java` (swerve,
Pigeon, intake, roller, centrífuga, shooter e o telescópio de bancada). Os
limites de corrente de cada mecanismo seguem as faixas dos fabricantes,
registradas em `MotorLimits.java`: CTRE (supply 20 a 80 A, 70 A caindo para
40 A após 1 s como ponto de partida, estator até 160 A e menor em mecanismo com
batente) e REV (NEO 40 a 60 A; Spark Flex até 60 A contínuos). `HardwareMapTest`
falha se algum ID repetir ou algum limite sair da faixa.

## O que é estimativa (medir no robô)

| Valor | Onde | Situação |
| --- | --- | --- |
| Ângulo de saída 62° e eficiência roda→bola 0,50 | `FuelConstants` | Escolhidos juntos para a curva de RPM da competição cair no hub de 1,5 a 5 m (arco de ~3 m de altura). **Medir** |
| Massa 0,21 kg, arrasto Cd 0,47 | `FuelConstants` | Estimativas |
| Boca do hub: 1,829 m de altura, raio 0,535 m | `ShooterConstants`, `FuelConstants` | **Medido** no modelo oficial do campo 2026 |
| Saída da bola a 0,61 m | `ShooterConstants.RELEASE_HEIGHT_METERS` | Medido no CAD do robô |
| Depósito 40 bolas, 5 bolas/s, pré-carga 8 | `FuelConstants` | Estimativas; conferir no manual e no robô |
| Rolete trabalhando a 1800 RPM ou mais | `FuelConstants.ROLLER_COLLECT_MIN_RPM` | ~88% da velocidade sem carga a 30%; conferir `Roller/Rpm` no robô |
| 5° por volta do motor no intake (72:1; 19 voltas = 95°) | `IntakeConstants` | Suposição para o desenho e a física |
| Massa do braço 3 kg | `IntakeConstants` | Estimativa |
| Carga da bola no rolete (0,6 N·m por 0,06 s) | `RollerConstants` | Escolhida para o detector funcionar na simulação |
| Queda de 6% do top por bola | `ShooterConstants` | Estimativa |

Achado da simulação: o voo leva ~1,0 s a 3 m e ~1,3 s a 4 m. O tiro andando
do código de competição estimava o tempo de voo como distância ÷ velocidade
(≈ 0,3 s): a compensação ficava curta e a bola caía ~0,45 m à frente, no
sentido do movimento (replay de 03/11/2026: 98% de acerto parado, ~5% acima
de 0,75 m/s). Desde 03/11/2026 `ShotCalculator.flightTimeSeconds` usa o
mesmo `BallisticModel` das bolas. No `ShotOnTheMoveTest` (2 a 4,5 m, quatro
sentidos), os tiros que caem no hub passaram de 42/48 para 48/48 a 0,5 m/s,
de 18/48 para todos a 1,0 m/s e de 3/48 para todos a 1,5 m/s. Se afastando
do hub a partir de ~4 m, o alvo virtual passa do alcance da curva e a
sequência trava em `LONGE_DEMAIS_CHEGUE_MAIS_PERTO`. Na simulação de
03/11/2026 15:43, já com a correção, os tiros andando a 0,25 m/s ou mais
acertaram 97 a 100% (antes: 74 a 87% e 0 a 9% acima de 0,75 m/s), e o erro
no sentido do movimento caiu de +0,45 m para −0,02 m. Os 22 erros que
sobraram foram todos longos, de 4,1 a 4,7 m, com o top +7% acima do alvo
(kV do NEO no Vortex). Com o RPM pedido, a bola cairia a 0,10 m do centro.
🚧 Falta medir o voo real (vídeo em câmera lenta) e conferir no robô; a aba
**BOLA 8** mostra o erro tiro a tiro.

## Cada tiro: planejado × sensores × onde chegou

A cada disparo detectado, o código grava duas trajetórias e a distância do
centro do hub em que cada uma cai:

| Chave | De onde vem | Robô real? |
| --- | --- | --- |
| `Fuel/Tiro/Planejado/...` | RPM **alvo** e ângulo **alvo** do pivô: onde a bola deveria ir | Sim |
| `Fuel/Tiro/Sensores/...` | RPM **medido** logo antes da bola e encoder do pivô | Sim |
| `Fuel/Tiro/Diferenca/...` | Sensores − planejado: velocidade de saída e direção | Sim |
| `Fuel/Simulacao/UltimoTiro/...` | Voo físico: **onde chegou** | Só simulação |
| `Fuel/Real/...` | Anotação da equipe: **onde chegou** | Sim |
| `Fuel/Tiro/Visao/...` | Limelight no tiro: pose já corrigida, correção recente (≤ 1 s), há quanto tempo, câmera | Sim (o erro da pose, só na simulação) |

No robô real nenhum sensor vê a bola. Depois de cada tiro, alguém clica no
AdvantageScope em `/Tuning/Fuel/AnotarTiroReal/` → **Acertou**, **ErrouCurto**,
**ErrouLongo** ou **ErrouDoLado**. O clique fica no log com o número do último
disparo detectado (`Fuel/Real/TiroAnotado`), e funciona também com o robô
desabilitado. Assim dá para ver, tiro a tiro na aba **BOLA 8**, se o
planejado dizia que ia acertar e o que aconteceu de fato.

**Visão no tiro.** A mira usa a pose do Drive. Se a Limelight não corrige há
tempo, a pose pode ter escorregado com as rodas, e o tiro erra mesmo com RPM e
ângulo certos. Antes de mexer na curva de tiro, olhe a aba **BOLA 10**: os
tiros errados tinham visão recente? Na simulação, a bola sai da pose
**verdadeira** (`Fuel/Tiro/Visao/PoseVerdadeira`) enquanto o código mira pela
estimada (`PoseEstimada`); a aba **BOLA 11** mostra as duas no campo.

Exemplo do teste automático (27 tiros a ~3,2 m): planejado a 0,26 m do centro,
sensores a 0,18 m, onde chegou a 0,22 m. Todos dentro da boca (0,535 m). A
diferença de velocidade (8,1 contra 7,7 m/s) é o top girando ~7% acima do alvo.

## Abas do layout

| Aba | O que mostra |
| --- | --- |
| BOLA 1 - Campo 3D | Robô com intake e shooter se mexendo, bolas no chão e no ar, trajetória prevista e último tiro |
| BOLA 2 - Robo de perto | O mesmo, com a câmera perto |
| BOLA 3 - Contagem | Sensores × simulação: coletas, disparos, acertos, erros, depósito |
| BOLA 4 - Intake | Ângulo, posição, alvo de segurar, tensão, corrente e limites |
| BOLA 5 - Roller e coleta | Corrente do rolete com o limite de detecção |
| BOLA 6 - Centrifuga e disparo | RPM da centrífuga e do top, queda de RPM com o limite |
| BOLA 7 - Tiros | Distância do centro do hub (último tiro e previsto), tempo de voo, velocidade de saída |
| BOLA 8 - Tiro planejado x onde chegou | Planejado, sensores e onde chegou, tiro a tiro, com a anotação real |
| BOLA 9 - Entradas | O que os IOs leram (o que o replay reproduz) |
| BOLA 10 - Visao em cada tiro | Segundos desde a correção da Limelight, erro da pose (simulação), tiros com e sem visão recente |
| BOLA 11 - Pose no tiro: estimada x verdadeira | Campo 2D com a pose que o código usou e a verdadeira (simulação) |
| BOLA 12 - Intake: real x modelo | Ângulo real × modelo em paralelo, diferença e resultado |
| BOLA 13 - Roller: real x modelo | RPM real × modelo; cada coleta aparece como queda do real |
| BOLA 14 - Roller: corrente real x modelo (cada bola) | Corrente medida × prevista: a diferença é a força de cada bola |
| BOLA 15 - Centrifuga: real x modelo | RPM real × modelo |
| BOLA 16 - Top: real x modelo (cada tiro) | RPM real × modelo, com os disparos detectados |
| BOLA 17 - Diagnostico dos motores | Execução, motor respondendo, resultado, comando e maiores diferenças |

O **modelo em paralelo** (padrão do telescópio 9458) é a física da simulação
recebendo a tensão que o motor aplicou de verdade. No robô real mostra se a
simulação está fiel; na simulação mostra o que o modelo não tem (as bolas).
Explicação completa: [ESTRUTURA_DO_CODIGO.md](../../../ESTRUTURA_DO_CODIGO.md#4-modelo-em-paralelo-real-x-simulado).
O painel `FRC9460-CicloDaBola` do Elastic mostra contagens, visão no tiro e
real × modelo ao vivo ([config/elastic](../../elastic/LEIA-ME.md)).

## Validação automatizada

`gradlew build` inclui:

- `IntakeTest`: começa fechado, abre até o soft limit, fecha, segura no meio contra a gravidade;
- `FuelSimulationTest`: coleta só com intake aberto e rolete puxando, depósito cheio,
  tiro da curva acerta de 3 m, roda lenta erra curto e a bola fica no campo;
- `FuelTrackerTest`: pico de corrente = uma coleta, partida do motor ignorada,
  queda de RPM com centrífuga = disparo;
- `cicloDaBolaTest` (JVM separada): o robô inteiro coleta e atira com um controle;
  os sensores precisam contar igual à simulação, toda chave dos layouts Shooter,
  CicloDaBola e do telescópio 9460 precisa existir no log, e todo tópico dos
  painéis do Elastic precisa existir com o tipo certo.

Para regenerar o layout e as posições das bolas, na raiz do projeto:

```powershell
python config/advantagescope/scripts/preparar_layout_ciclo_da_bola.py
```

```powershell
python config/advantagescope/scripts/extrair_bolas_campo_2026.py
```
