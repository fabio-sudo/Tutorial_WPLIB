# SysId do telescópio 9458

Objetivo: medir **kS, kG, kV e kA** do telescópio real e calcular kP/kD.
Os resultados saem em volts e **metros da ponta**, as mesmas unidades de
`REAL_K*` em `TelescopioConstants_9458.java`.

Código: `src/main/java/frc/robot/calibration/Telescopio9458SysId.java`.

## Botões — controle 0, modo Test (segurar)

> **Desde 22/10 (noite), R1/L1 no modo Test fazem passos de 10 cm** (R1 sobe e L1
> desce). Para calibrar de novo, em `RobotContainer.java` troque
> `telescopio9458.stepUpWhileHeldCommand()` por `telescopio9458SysId.quasistaticUpAndDown()`
> e `telescopio9458.stepDownWhileHeldCommand()` por `telescopio9458SysId.dynamicUpAndDown()`.
> A tabela abaixo vale com o SysId religado.

O telescópio **sempre começa fechado**. Cada botão sobe, pausa 1 s em cima e
desce, terminando fechado de novo. Dois apertos geram os 4 testes do SysId.

> **Desde 28/10/2026 o 9458 está fora de uso:** o subsistema, o painel e o SysId dele estão **comentados** no `RobotContainer` (cabeçalho "ANTIGO"), então nada dele roda no robô 9460. Os botões (RT/LT no Teleop e R1/L1 no modo Test) também. Para voltar, descomente os blocos e os imports; a tabela abaixo é o que eles fazem.
Para rodar o SysId, descomente o bloco do modo Test e troque os comandos como
indicado nele.

| Segurar | Teste | O que acontece |
|---|---|---|
| **R1 (RB)** | Quasistático | Rampa de 0,5 V/s subindo até 0,846 m, pausa e rampa descendo até 0,05 m. Cerca de 12 s. |
| **L1 (LB)** | Dinâmico | Degrau de +2 V subindo, pausa e degrau de −2 V descendo. Cerca de 10 s. |

- Os botões **só funcionam no modo Test** e **só enquanto estão segurados**. Ao
  soltar, o teste é cancelado e o motor vai para 0 V. Ele não continua sozinho.
- Sem BACK e sem START. BACK + R1/L1 é o SysId do swerve, e START + R1/L1 é a
  calibração do steer. R1 e L1 juntos não fazem nada.
- Mesmo segurando, cada parte para sozinha na parada do curso, depois de 10 s
  ou **se travar**. Travar significa mais de 22 A com o telescópio parado por
  1 s; nesse caso, o resto da sequência não roda.
- Durante o teste não há PID. Desabilitar, posição inválida e falha de comando
  continuam cortando o motor, e os soft limits do Talon continuam ativos.

Valores ajustáveis em `TelescopioConstants_9458.java`, região **SYSID**.

## Antes de começar

1. **Pendrive no roboRIO.** O AdvantageKit grava o log em `/U/logs`. Sem pendrive,
   não há arquivo para o SysId.
2. **Sentido do motor.** Em Teleop, peça uma meta pequena no painel
   (`/Tuning/Telescopio9458/MetaMetros`, com `ControleAtivo`). O telescópio
   precisa **abrir**. Se fechar, corrija `MOTOR_INVERTED` antes do SysId.
3. Ligue o robô com o telescópio **fechado** (é o zero). No AdvantageScope,
   `Telescopio9458/Diagnostico` deve estar `OK`.
4. Deixe a área do telescópio livre.

## Na bancada

1. Driver Station: modo **Test**, **Enable**.
2. Segure **R1** até o telescópio subir, pausar, descer e parar fechado.
3. Segure **L1** até subir, pausar, descer e parar fechado.
4. **Disable.**

Para conferir no AdvantageScope:

- `Telescopio9458/SysId/Teste`: parte que rodou (ex.: `QUASISTATICO DESCENDO`).
- `Telescopio9458/SysId/Resultado`: o esperado é `CHEGOU_NA_PARADA`.
  - `CANCELADO`: o botão foi solto.
  - `TRAVOU`: corrente alta com o telescópio parado.
  - `TEMPO_ESGOTADO`: não chegou à parada em 10 s.
- `Telescopio9458/Estado`: `SYSID` durante o teste.

**Solte o botão imediatamente** se houver barulho, se o telescópio travar ou
se ele andar no sentido errado.

Se soltar no meio, o telescópio fica onde parou. Para voltar a fechar, segure
o mesmo botão de novo (ele completa a subida e desce) ou feche com **LT** em
Teleop. Partes canceladas também ficam no log. O resultado mais limpo vem de
um log só com os 2 apertos completos: em caso de dúvida, reinicie o código
(novo log) e faça R1 e L1 de novo.

## Baixar o log

O mesmo teste fica gravado em dois lugares. Use qualquer um.

**A. Log do AdvantageKit (precisa de pendrive).** No AdvantageScope:
**File > Download Logs**, conectado ao roboRIO da 9458 (**10.94.58.2**). Ou tire
o pendrive e copie a pasta `logs`. Sem pendrive, a Driver Station mostra
`[AdvantageKit] Failed to open output log file` e esse arquivo não existe.

**B. Log da CTRE (funciona sem pendrive).** O roboRIO grava um `.hoot` sozinho
em `/home/lvuser/logs`. No **Phoenix Tuner X**, use o extrator de logs para
baixar o `.hoot` do horário do teste e exporte para `.wpilog`.
Em cada partida do robô nasce um `.hoot` novo (`rio_<data>_<hora UTC>.hoot`;
a hora UTC é a de Brasília + 3 h). Pegue o do horário em que os botões foram
apertados.

| Campo no SysId | A. AdvantageKit | B. CTRE (Tuner X) |
|---|---|---|
| Test State | `/RealOutputs/Telescopio9458/SysIdTestState` | `Telescopio9458/SysIdTestState` |
| Position | `/Telescopio9458/Inputs/ExtensionMeters` | `Telescopio9458/PosicaoMetros` |
| Velocity | `/Telescopio9458/Inputs/VelocityMetersPerSec` | `Telescopio9458/VelocidadeMetrosPorSeg` |
| Voltage | `/Telescopio9458/Inputs/AppliedVolts` | `Telescopio9458/TensaoVolts` |

Não use `RobotMode` como Test State. Ele só tem `Disabled`/`Test`/`Teleop`, e o
SysId responde "No tests found". Também não use o `SysIdTestState` da raiz,
que é do swerve. Os campos `Phoenix6/TalonFX-5/...` estão em voltas do rotor,
não em metros.

**C. Último recurso: CSV da gravação ao vivo.** Se o AdvantageScope estava
conectado durante o teste, use **File > Export Data** (CSV) e converta com:

```
python -I config/advantagescope/scripts/sysid_9458_csv_para_wpilog.py log.csv sysid.wpilog
```

O script descobre os testes pelo `Estado`/`Comando`, grava os campos da coluna A
e já mostra os ganhos calculados.

## Analisar no SysId

1. Abra o **SysId** (ferramentas da WPILib) e carregue o `.wpilog` no **Log Loader**.
2. Arraste os campos da tabela acima para o **Data Selector**. Unidade: **Meters**.
3. Análise: **Elevator**. O SysId mostra kS, kV, kA e **kG**.
4. Confira os gráficos. Os pontos do quasistático devem formar uma reta.
5. **kA deve sair próximo de 0 ou instável.** Isso é esperado: os sinais são lidos
   a cada 20 ms e o motor acelera em cerca de 2 ms, então a aceleração quase não
   aparece. Se sair estranho, use 0. Ele pesa menos de 0,02 V no controle.
6. **Feedback**: preset **WPILib (2020-)**, laço de **posição** (Loop Type = Position).
   Não use o preset **CTRE Phoenix 6**: ele serve para PID rodando dentro do Talon
   (Slot0, 1 ms, em voltas), e aqui o PID roda no roboRIO, em metros da ponta. O controle roda
   no roboRIO a cada 20 ms. Comece com erro máximo de posição de cerca de 1 cm
   e esforço máximo de **6 V** (`MAX_CONTROL_VOLTS`). O SysId sugere kP e kD em V/m.

## Resultado de 22/10/2026 (robô real, bancada 9458)

Dados: CSV da gravação ao vivo (opção C), com 2 apertos completos de R1 e 2 de L1.
O telescópio parou em 0,846 m na subida e em cerca de 0,05 m na descida, como previsto.
Ajuste com o modelo Elevator do SysId:

| | kS (V) | kG (V) | kV (V/(m/s)) | kA (V/(m/s²)) | R² |
|---|---|---|---|---|---|
| Todos os testes | **0,042** | **0,201** | **9,36** | ≈ 0 | 0,9995 |
| Só quasistático | 0,047 | 0,202 | 9,35 | — | 0,9997 |
| Teórico (5 kg, sem atrito) | 0 | 0,168 | 9,40 | 0,017 | — |

- O kV real ficou igual ao teórico. Isso confirma a redução 16:1, a coroa de 16
  dentes e a cascata 2×.
- O kG real é cerca de 20% maior que o teórico: a carga equivale a uns 6 kg na ponta.
- O kS de cerca de 0,04 V indica pouco atrito.
- O kA não aparece em amostras de 20 ms. Use 0 ou mantenha o teórico, 0,017.

**Gravado no código em 22/10:** `REAL_KS = 0.042`, `REAL_KG = 0.201`,
`REAL_KV = 9.36` e `REAL_KA = 0`. Em 4 rodadas seguidas, kG ficou entre 0,201 e 0,211
e kV entre 9,27 e 9,36.

**kP/kD em teste:** `REAL_KP = 35` e `REAL_KD = 0`. Esses valores saem do LQR de posição
(erro máximo de 5 cm, 6 V, 20 ms) considerando o **atraso de leitura de 20 a 40 ms**
estimado pelo SysId (Position Measurement Delay).

- **kP 55 / kD 3,7** (LQR sem atraso): o telescópio ficou **pulando**. Cerca de 2,6%
  das leituras do encoder chegam repetidas, e o kD transformava isso em solavancos
  de ±0,4 V, com picos de 2,4 V.
- **kP 18** (anterior): sem tremor, mas o telescópio seguia o perfil com 1 a 1,5 cm
  de atraso, passava cerca de 1 cm da altura ao soltar o botão e, depois de descer,
  parava uns 6 mm abaixo da meta.

No SysId, preencha o **Measurement Delay** (ms) do Feedback Analysis com cerca de 20.
Sem isso, o kP sugerido sai alto demais.

## Aplicar os resultados

1. **Testar sem recompilar**: com o robô **desabilitado**, preencha
   `/Tuning/Telescopio9458/PID/kP…kA` no AdvantageScope e marque
   `AplicarGanhos`. `Telescopio9458/Ajuste/Resultado` deve mostrar
   `GANHOS_APLICADOS`. kS ou kG acima de 6 V são recusados.
2. Habilite em Teleop e teste com RT/LT. Ao soltar, o telescópio deve parar
   e segurar a altura sem cair nem oscilar.
   `Telescopio9458/PID/FeedforwardVolts` deve responder pela maior parte da
   tensão, e `PID/FeedbackVolts` deve ficar pequeno.
3. Quando estiver bom, grave os valores em `REAL_KS`, `REAL_KG`, `REAL_KV`,
   `REAL_KA`, `REAL_KP` e `REAL_KD` (`TelescopioConstants_9458.java`).

## Treinar na simulação

Dá para ensaiar o procedimento inteiro antes de usar o robô:

1. **WPILib: Simulate Robot Code** com a Simulation GUI.
2. Arraste o controle para **Joysticks [0]** e escolha o modo **Test**.
3. Segure R1 e depois L1. O log fica em `logs/simulacao`.
4. No SysId, o resultado deve ser kG ≈ 0,168 V, kV ≈ 9,40 V/(m/s), kS ≈ 0
   (a simulação não tem atrito) e kA ≈ 0. Se ficar muito diferente, o problema
   está na escolha dos campos ou das unidades, não no robô.

O teste automático `Telescopio9458SysIdTest` faz esse ajuste com os dados de R1 e
L1 e recupera kG = 0,1676 V e kV = 9,397 V/(m/s), iguais à física simulada.
