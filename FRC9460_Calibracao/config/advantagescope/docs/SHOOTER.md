# Shooter — simulação e AdvantageScope

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

- Curso: ±6 voltas do motor = ±90° (limite da fiação). O Talon bloqueia além disso.
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

| Arquivo | Papel |
| --- | --- |
| `ShooterConstants` | Todos os números (IDs, curso, ganhos, curva, estimativas da simulação) |
| `ShotCalculator` | Contas puras: alvo da aliança, mira do pivô, RPM por distância, tiro andando |
| `Shooter` | Subsistema: lê os IOs, protege, grava telemetria, oferece os comandos |
| `ShooterTrackAndShootCommand` | Sequência: mira → acelera → alimenta só com tudo pronto |
| `ShooterPivotIO` / `...TalonFX` / `...Sim` | Motor que gira o shooter: contrato, robô real, física |
| `ShooterWheelIO` / `...SparkFlex` / `...TalonFX` / `...Sim` | Rodas por velocidade: top, tower, centrífuga |
| `ShooterVisualizer` | Desenho 3D (peças do modelo), visão de cima e linhas de mira |
| `ShooterDashboard` | Botões em `/Tuning/Shooter`, para testar sem segundo controle |

## Testar na simulação

1. **WPILib: Simulate Robot Code** com a Simulation GUI.
2. Na Simulation GUI, em **FMS**, escolha a aliança (Blue 1 mira no hub azul).
3. **Teleoperated** e **Enabled**. Dirija com o controle em **Joysticks [0]**.
4. AdvantageScope: **File > Connect to Simulator**, importe o layout.
5. Para atirar, use um dos dois:
   - um segundo controle em **Joysticks [1]**: **RB** segurado = mirar e atirar;
   - no AdvantageScope, ative `/Tuning/Shooter/MirarEAtirar` (ou `MirarNoHub`).
6. Gire o robô com o hub **atrás** dele: o shooter aponta para trás no zero e
   só alcança ±90°. Com o hub na frente, o estado fica
   `HUB_FORA_DO_CURSO_GIRE_O_ROBO` e ele não atira.

Também há um log pronto, gravado com o robô inteiro (Drive + câmeras virtuais +
shooter): `logs/simulacao/Shooter_Demonstracao.wpilog`. Nele o robô gira, atira,
anda, o pivô acompanha o hub e chega ao limite de 90°.

## Controle do operador (porta 1)

| Entrada | Ação |
| --- | --- |
| RB segurado | Mirar no hub, acelerar e atirar quando pronto |
| LB segurado | Só mirar (rodas paradas) |
| A segurado + analógico direito | Pivô manual lento (teste de sentido) |
| BACK + START, desabilitado | Centro atual vira o zero |

## Abas do layout

| Aba | O que mostra |
| --- | --- |
| SHOOTER 1 - Campo 3D | Robô com o shooter girando, linha laranja (para onde aponta), verde (até o alvo), pose da Limelight |
| SHOOTER 2 - Robo de perto | O mesmo, com a câmera perto do robô |
| SHOOTER 3 - Campo 2D | Visto de cima, com as duas linhas |
| SHOOTER 4 - Visto de cima | Desenho do robô: frente, limites de ±90°, shooter e direção do hub |
| SHOOTER 5 - Pivo | Ângulo, alvo, ângulo necessário, limites, tensão e corrente |
| SHOOTER 6 - Rodas (RPM) | RPM medido e alvo de top (dois motores), tower e centrífuga |
| SHOOTER 7 - Sequencia de tiro | Estado da sequência e cada condição de pronto |
| SHOOTER 8 - Mira e Limelight | Erro de direção, distância e correções aceitas da visão |
| SHOOTER 9 - Entradas | Tabela do que os IOs leram (o que o replay reproduz) |
| SIM SHOOTER - Zero manual | Ângulo do encoder × ângulo real (só simulação) |

Quando as linhas laranja e verde ficam sobrepostas, o shooter está apontado.
Se nada se mexe no 3D, confira se a fonte **Component** está logo abaixo do
**Robot** (é a lista `Mecanismos/Componentes`: model_0 = intake, model_1 = shooter).
O intake ainda não tem subsistema e fica na posição do CAD.

## Diferenças em relação ao código de competição

- O alvo segue a aliança (antes era sempre o hub azul).
- Com o hub fora do curso de ±90°, não atira (antes atirava com o pivô no limite,
  na direção errada).
- Ajustes da mira que ficavam no SmartDashboard viraram constantes: assim o
  replay repete exatamente a mesma conta.
- Perda do zero por reset do Talon agora é detectada.
- O "pronto por tempo" (0,75 s) foi mantido, mas fica marcado em
  `Shooter/Sequencia/ProntoPorTempo`. Para exigir o RPM, use
  `USE_SPINUP_TIME_FALLBACK = false`.

## O que conferir no robô (não dá para provar na simulação)

- **Redução do pivô:** o código assume 6 voltas = 90° (24:1). Contar voltas.
- **Posição do shooter:** o código de competição usa 5 cm para trás e 14 cm
  para a **direita**; o CAD mede 14,4 cm para trás e 14,5 cm para a **esquerda**.
  Medir no robô e corrigir `SHOOTER_OFFSET_*`.
- **Sentido:** `PIVOT_DIRECTION_SIGN = 1` (o padrão do SmartDashboard antigo).
  Com o robô levantado, ângulo positivo deve girar anti-horário visto de cima.
- **Motores e kV:** o kV dos Spark Flex usa a velocidade do NEO. Se o top for
  NEO Vortex (como no CAD), a simulação (sem atrito) mostra o top uns 7% acima
  do alvo, sem entrar na tolerância: o tiro sai pelo tempo. No robô o atrito
  muda isso; olhar o RPM real na aba **SHOOTER 6** no primeiro teste.
- **Altura do alvo:** 2,49 m veio do código antigo; conferir no manual do jogo.
- Inércias e motores da simulação são estimativas (`ShooterConstants`, região SIMULAÇÃO).

## Validação automatizada

`gradlew build` inclui:

- `ShotCalculatorTest`: curva igual à da competição, centros dos hubs, mira e tiro andando;
- `ShooterPivotIOSimTest`: Motion Magic, soft limits, batente da fiação e zero manual errado;
- `ShooterTest`: espera a Limelight, atira com o hub atrás, gira 45°, não atira fora do curso;
- `shooterLogTest` (JVM separada): grava um tiro pelo painel e confere que toda
  chave do shooter usada por este layout existe no log.

Para regenerar o layout, na raiz do projeto:

```powershell
python config/advantagescope/scripts/preparar_layout_shooter.py
```
