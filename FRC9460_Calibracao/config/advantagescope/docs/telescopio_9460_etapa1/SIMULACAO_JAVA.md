# Telescópio 9460 — simulação Java

A simulação do telescópio faz parte do mesmo programa do swerve e da visão.
O Java calcula o movimento a partir da tensão, massa e gravidade; o AdvantageScope
mostra os dados e os componentes do modelo existente. A base permanece na origem
da bancada: ainda não foi definida sua posição de instalação no robô completo.

## Executar e controlar

1. No VS Code, execute **WPILib: Simulate Robot Code** e selecione **Simulation GUI**.
2. No AdvantageScope, conecte a **NetworkTables 4 (AdvantageKit)** em `127.0.0.1`.
3. Importe `config/advantagescope/layouts/Telescopio9460-Simulacao.json`.
4. Se o modelo não aparecer, selecione a pasta `config/advantagescope/mecanismos`
   em **App > Use Custom Assets Folder...**. O asset mantém o nome
   **FRC9460 - Telescopio - Etapa 1**, para preservar os layouts anteriores.
5. Na Simulation GUI, selecione **Teleoperated** e **Enabled**.
6. No AdvantageScope, ative o **Tuning Mode** pelo ícone de ajustes ao lado da busca.
   Expanda **Tuning > Telescopio** e altere os campos abaixo.

| Campo | Como usar |
| --- | --- |
| `MetaMetros` | Extensão da ponta desde a posição recolhida: de 0 a 1,252 m. Por exemplo, 0,626 m é metade do curso. |
| `ControleAtivo` | `true` inicia o controle; `false` cancela o comando e pede tensão zero. |
| `CicloAutomatico` | Com `ControleAtivo=true`, repete metade → aberto → recolhido, com pausas de 1 s. |

Para experimentar uma posição, deixe `CicloAutomatico=false`, ajuste `MetaMetros`
e ligue `ControleAtivo`. Para a demonstração, ligue os dois booleanos.
As edições numéricas são enviadas ao pressionar Enter ou sair da caixa.
Os campos editáveis ficam em **Tuning**, não em **NetworkInputs** ou **RealOutputs**.
Os booleanos aparecem como círculos vermelho/verde na árvore de campos.

Os comandos do painel só operam em teleoperado habilitado.
O modelo físico continua evoluindo em Disabled; a tensão é zerada e a gravidade
pode recolher o mecanismo. Ao voltar para Enabled, um `ControleAtivo` ainda ligado
reaplica a meta do painel ou reinicia o ciclo. Desligue esse campo para encerrar o ensaio.

**Parar e manter posição são ações diferentes:** `ControleAtivo=false` pede 0 V.
Para manter uma extensão contra a gravidade, deixe o controle ativo com a meta desejada.

## Testar com o controle Xbox

| Botão (segurar) | Ação |
| --- | --- |
| **L1 (LB)** | Abre até o fim de curso (1,252 m). |
| **L2 (LT)** | Fecha até o batente (0 m). |
| Soltar | O perfil freia (cerca de 13 cm a 0,4 m/s) e mantém a altura contra a gravidade. |

1. Conecte o controle no computador. Na Simulation GUI, arraste-o de
   **System Joysticks** para **Joysticks [0]**.
2. Importe `config/advantagescope/layouts/Telescopio9460-TesteControle.json`:
   3D, 2D, aba do controle, posição com as faixas `Comando` e `Estado`, motor e velocidade.
3. Selecione **Teleoperated** e **Enabled** e segure L1 ou L2.

L1 e L2 só movem o telescópio em teleoperado habilitado e sem BACK ou START:
BACK + L1/L2 é o SysId do swerve e START + L1/L2 é a calibração do steer.
L1 e L2 juntos não movem. Um botão interrompe o comando do painel; para voltar
ao painel, desligue e ligue `ControleAtivo`.

Até 28/10/2026 eram RB (abre) e LB (fecha). R1 agora sobe o intake; o bloco
antigo está comentado no `RobotContainer`, com o cabeçalho "ANTIGO".

## O que foi implementado

| Arquivo Java | Função |
| --- | --- |
| `TelescopioConstants` | Geometria, hipóteses físicas, limites do perfil e ganhos iniciais. |
| `TelescopioIO` | Contrato de leitura dos sensores e atuação por tensão. |
| `TelescopioIOSim` | Kraken X44 e `ElevatorSim`; uma atualização de 20 ms por ciclo. Também é o modelo em paralelo (real x simulado). |
| `Telescopio` | Perfil de velocidade/aceleração, PID e compensação de gravidade; proteções, logs, modelo em paralelo e diagnóstico. |
| `commands/telescopio/*` | `TelescopioAbrirCommand` (RB), `TelescopioFecharCommand` (LB), `TelescopioIrParaCommand`, `TelescopioSeguirMetaCommand` (painel) e `TelescopioCicloCommand` (ciclo automático). |
| `TelescopioVisualizer` | Mechanism2d e poses 3D compatíveis com os assets existentes. |
| `TelescopioDashboard` | Três entradas de painel registradas pelo AdvantageKit, inclusive no replay. |
| `TelescopioIOTalonFX` | Motor real (TalonFX, `VoltageOut`). Só é criado quando `CanIds.TELESCOPIO_9460` tiver um ID; até lá o robô real usa um IO vazio e mostra um aviso. |

As fontes ficam em `src/main/java/frc/robot/subsystems/telescopio`.
`RobotContainer` cria o subsistema e conecta o painel. No replay, seleciona um IO
vazio: os sensores vêm do log, sem executar novamente a física.

## Medidas e hipóteses

- Altura aberta na posição do CAD: **2,1145 m**.
- Cursos adotados da montagem visual: **N2 = 0,624 m**, **N3 relativo = 0,628 m**.
- Extensão máxima da ponta: **1,252 m**.
- Altura recolhida reconstruída: **0,8625 m**. A medida informada de 0,866 m difere
  em 3,5 mm; não foi apresentada como uma nova medição física.
- Movimento simultâneo: ambos percorrem a mesma **fração** de seu curso. Na metade,
  N2 está em 0,312 m e N3 em 0,314 m. A ponta soma os dois deslocamentos.
- Motor ideal: **1 Kraken X44**, modelo sem FOC; redução **5:1**, tambor **35 mm**.
- Massa equivalente concentrada na ponta: **5 kg**.
- Relação provisória ponta/cabo: **1:1**, configurável em `SIM_CABLE_TO_TIP_RATIO`.
  O CAD não contém o caminho completo do cabo; a relação não foi inferida das polias.
- O modelo não representa massas separadas por estágio, atrito, elasticidade do cabo
  ou queda de bateria causada pelo consumo. Ele respeita a tensão da bateria simulada,
  calcula a corrente nos enrolamentos e imita o limite de corrente do estator do TalonFX.

O controle usa perfil inicial de **0,4 m/s**, **0,6 m/s²**, tensão limitada a **6 V**
e tolerância de posição de **5 mm**. O feedforward é calculado a partir do modelo
ideal; esses valores não são uma calibração do mecanismo real.

No roboRIO, o telescópio permanece indisponível. Para implementar a parte física,
faltam ID/barramento CAN, inversão, referência de posição, sensores de limite,
ligação dos cabos e parâmetros validados. Nenhum ID foi inventado.

## Proteções do motor

| Proteção | Onde | Valor |
| --- | --- | --- |
| Batente superior | `Telescopio.limitAtEnds` | Lendo 1,252 m, a tensão para cima fica limitada ao kG (só segura o peso). |
| Batente inferior | `Telescopio.limitAtEnds` | Recolhido, 0 V; nunca empurra para baixo. |
| Travamento | `Telescopio` (Debouncer da WPILib) | ≥ 30 A e < 0,02 m/s por 0,25 s: para de empurrar e segura onde travou. Estado `TRAVADO`. |
| Limite de corrente do estator | TalonFX e `TelescopioIOSim` | 40 A. |
| Limite de corrente da bateria | TalonFX | 30 A; cai para 20 A depois de 1 s. |
| Soft limits | TalonFX | 57,4 voltas do rotor para cima (1,262 m) e -0,45 para baixo (-1 cm). |
| Neutro | TalonFX | Brake: freia ao desabilitar. |

- **Destravar:** solte o botão e aperte de novo (ou desligue e ligue `ControleAtivo`).
  Desabilitar também limpa o travamento.
- **Por que 40 A:** a CTRE sugere 120 A como ponto de partida comum (80-160 A) e
  avisa que mecanismos com batente podem precisar de um limite menor.
  - Na simulação, segurar 5 kg usa cerca de 12 A, e o ciclo completo não passa de 18 A.
  - 40 A deixa folga de cerca de 3x e limita a força do motor na ponta a uns 170 N.
- **Por que 30 A na bateria:** é o exemplo de elevador da CTRE.
- **Ao definir o ID CAN** (`CanIds.TELESCOPIO_9460` e a lista `ALL`): o
  `TelescopioIOTalonFX` aplica `createConfiguration()` e zera a posição ao ligar.
  - O telescópio precisa estar **recolhido** quando o robô liga (zero manual).
  - Se o Talon reiniciar, o zero se perde: a posição fica inválida e o motor para.
  - Conferir `TelescopioConstants.MOTOR_INVERTED` (positivo deve estender) sem carga.
  - Os soft limits dependem da relação cabo/ponta provisória (1:1).
- **Validar no mecanismo real:** limites de corrente e o limiar de travamento.

No AdvantageScope: `/Telescopio/Inputs/StatorCurrentLimitActive`,
`/RealOutputs/Telescopio/Protecao/Travado`, `LimiteTopoAtivo` e
`LimiteCorrenteEstatorAmps` (aba **Motor - corrente e protecoes** do layout de teste).

Fontes: [limites de corrente da CTRE](https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html),
`CurrentLimitsConfigs` e `SoftwareLimitSwitchConfigs` do Phoenix 6 26.3.0,
`DCMotor.getKrakenX44` da WPILib 2026.2.1 (4,11 N·m, 279 A, 7758 rpm, dados do dinamômetro da CTRE)
e [Debouncer da WPILib](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/debouncer.html).

## Layout e logs

O layout novo contém mecanismo 2D, modelo 3D, cursos simultâneos, meta/medida/perfil,
tensão/corrente e velocidade. As chaves antigas de visualização foram preservadas:

- `/RealOutputs/Telescopio/Mecanismo2d`
- `/RealOutputs/Telescopio/BasePose`
- `/RealOutputs/Telescopio/Componentes`
- `/RealOutputs/Telescopio/ExtensaoN2Metros`
- `/RealOutputs/Telescopio/ExtensaoRelativaN3Metros`
- `/RealOutputs/Telescopio/ExtensaoVisualMetros`

As leituras ficam em `/Telescopio/Inputs`. `PositionValid` informa se a extensão
tem referência válida. As saídas incluem `MetaMetros`, `TensaoSolicitadaVolts`,
`Estado`, `ControleAtivo`, `NaMeta` e `VisualizacaoValida`.
Em replay, as saídas recalculadas ficam em `/ReplayOutputs`.

**Real x modelo em paralelo** (padrão do telescópio 9458): a física recebe a
tensão medida no motor e prevê a extensão. As abas **Real x modelo - posicao**,
**Real x modelo - corrente** e **Diagnostico do motor** (nos dois layouts) usam
`/RealOutputs/Telescopio/Comparacao/...` e `/RealOutputs/Telescopio/Diagnostico/...`.
Com o telescópio e o modelo parados, o modelo se realinha e o erro do movimento
fica em `DiferencaAoPararCentimetros`. No Elastic: painel `Telescopio9460`
([config/elastic](../../../elastic/LEIA-ME.md)).

A simulação grava em `logs/simulacao/Simulacao_aa-MM-dd_HH-mm-ss.wpilog`: o nome
já identifica a simulação em **File > Open Log(s)**. O replay grava ao lado, com `_replay`.
Os testes geram um exemplo de **20 segundos calculados pela simulação**, em:

`build/telescopio-validation/Telescopio9460-Simulacao.wpilog`

Abra esse arquivo com **File > Open Log(s)...** para assistir sem iniciar o robô.
O teste executa o ciclo automático, muda para uma meta de 0,31 m, interrompe o
controle e termina em Disabled. O arquivo `Telescopio9460-Replay.wpilog`, ao lado,
permite comparar as saídas originais e recalculadas.

Os arquivos antigos `Telescopio_9460-Demo.wpilog` e `Telescopio-Etapa1-Layout.json`
continuam sendo a demonstração visual anterior, com movimentos sequenciais.
Não são a simulação Java descrita aqui.

## Verificação

Na raiz do projeto, com o Java da WPILib:

```powershell
.\gradlew.bat test --tests "frc.robot.subsystems.telescopio.*"
.\gradlew.bat telescopioReplayTest
.\gradlew.bat build
```

`telescopioReplayTest` gera primeiro o log físico e depois recalcula o controle em
outro processo. A comparação verifica os sensores/painel gravados, a igualdade
das tensões calculadas e a preservação do arquivo original. `check`/`build` incluem
esses testes, além das verificações anteriores do projeto.

Referências: [simulação física WPILib](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/physics-sim.html),
[entradas de painel AdvantageKit](https://docs.advantagekit.org/data-flow/recording-inputs/dashboard-inputs/),
[Tuning Mode no AdvantageScope](https://docs.advantagescope.org/overview/live-sources/tuning-mode/).
