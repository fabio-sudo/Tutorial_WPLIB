# Controles e mecanismos — 29/10/2026

## Piloto (porta 0)

Todos funcionam segurando o botão, em Teleop habilitado, com BACK e START soltos.

| Botão | Ação | Ao soltar |
|---|---|---|
| R1 / RB | Desce/abre intake | Segura a posição |
| L1 / LB | Sobe/fecha intake | Segura a posição |
| R2 / RT | Liga roller de coleta | Para o roller |
| L2 / LT | Pivô mira automaticamente no hub | Para o pivô |
| B | Sequência: mira, top/flywheel, tower e centrífuga (só para a frente) | Para/freia os mecanismos |

R1 e L1 juntos cancelam as duas direções. B tem prioridade sobre L2. A centrífuga
só alimenta depois das verificações de pose, alcance, sensores, mira e rodas da
sequência; B não ignora essas verificações. Desabilitar cancela os comandos.

As ligações estão em `controls/DriverMechanismBindings`, chamadas pelo
`RobotContainer`. Cada Command exige seus subsistemas. Os mapeamentos antigos
de elevadores, toggles B/START+B e calibração/SysId em START/BACK com
bumpers/gatilhos foram mantidos comentados no `RobotContainer`. O segundo
controle e as demais calibrações continuam com suas funções existentes.

**Testes de calibração do drive (desde 29/10/2026, só com BACK segurado):**
BACK + Y tensão baixa (era A sozinho), BACK + ↑ varredura, BACK + → kS,
BACK + ← kS ao contrário. Antes um toque em A ou numa seta fazia o robô andar.
Operador: BACK + START zera o pivô e BACK + Y zera o intake (desabilitado).
Piloto, **START sozinho**: não faz nada desde 31/10/2026. Nenhum botão aciona a centrífuga
sozinha: ela já funciona e só gira pela sequência de tiro. A lógica do teste sem trava
(`Centrifuga.openLoopTestCommand`) e o log `Centrifuga/TesteSemTrava` continuam no código.
Start + Y/A/↓ continuam como antes.

**B sem reversão (29/10/2026, depois do teste):** o B não reverte mais a centrífuga
antes do tiro. Soltou o B sem a centrífuga ligar (mais de 1 s), a Driver Station
mostra o último estado e o erro do pivô (`Shooter/Sequencia/Detalhe`).

## Intake: rotor, redução e limite

Fonte: `C:/Users/Admin/Desktop/tuner-plot.csv`, 10.758 linhas, 123,147 s de
janela. CAN 35 direito, movimentação manual informada pela equipe. Há 2.613
amostras de RotorPosition e 8.278 de Position, em linhas parcialmente
intercaladas. Os dois sinais vão de −0,000977 a −22,205078 voltas; não são
dados de corrente ou esforço no batente.

SHA-256 do CSV: `9f37dbef9b06940a17539f0fb85a50d6a27d362fed13ebf7c2dd6bcbb19b1897`.

- Primeiro estágio: **9:1**; segundo: **9:1**; total por motor: **81:1**.
- Cada volta do rotor = 1/81 volta da articulação = **4,444444°**.
- Curso máximo de rotor registrado: **22,205078 voltas** (98,689236°).
- Extremo de abertura de referência: **−22,205078 voltas**.
- Margem antes desse extremo: **0,50 volta de rotor**, ou **2,222222°**.
- Soft limit de abertura no líder: **−21,705078 voltas**.
- O software começa a parar em aproximadamente **−21,605078** (buffer extra
  de 0,10 volta). O valor final depende da velocidade, do hold e da mecânica.
- Fechamento: referência 0 ao ligar fechada, corte antecipado de software em
  **−0,10 volta**, com soft limit de respaldo em 0.

A magnitude do curso direito foi aplicada ao sentido de abertura já usado no
líder (negativo). No primeiro teste físico, conferir o sinal e o zero de ambos
os encoders com movimento curto, antes de percorrer o curso inteiro. O CSV
registra o extremo observado, não certifica a distância de frenagem sob carga.

Os números ficam em `IntakeConstants`. A redução não depende do limite e não é
aplicada duas vezes no Talon: o IO e os soft limits continuam em **voltas do
rotor**, mantendo SensorToMechanismRatio = 1. A conversão angular e o modelo
simulado usam 81:1. O batente da física fica depois do limite operacional,
preservando a margem no modelo.

Campos adicionais no AdvantageScope:

- `Intake/ReducaoTotal`, `Intake/VoltasSaida`;
- `Intake/LimiteMecanicoVoltasRotor`, `Intake/LimiteOperacionalVoltasRotor`;
- `Intake/MargemSegurancaGraus`.

O intake deve iniciar fechado. Reiniciar o programa também refaz o zero de
partida: se estiver aberto, fechar/desabilitar e refazer a referência antes
de movimentar. A alteração não instala um fim de curso físico.

## Elevadores exclusivos de simulação

IDs futuros reservados no mapa único: **50 (9458)** e **51 (9460)**. São
distintos dos IDs atuais do robô. `CanIds.ELEVATORS_HARDWARE_ENABLED = false`
impede que as fábricas dos dois mecanismos criem TalonFX no REAL, inclusive
se um painel pedir movimento. Trocar números no código não reprograma IDs
de dispositivos físicos. SIM e REPLAY continuam disponíveis. O elevador
9460 pode ser testado pelo painel, sem ocupar L1/L2; a instância do 9458 no
RobotContainer já estava arquivada antes desta alteração.

## Centrífuga: RPM em malha fechada

O CAN 36 permanece. Já havia PID de velocidade; a alteração acrescenta
**MotionMagicVelocityVoltage**, perfilando a referência no próprio TalonFX
com aceleração inicial de **60 rot/s² (3.600 RPM/s)**, jerk 0 e pedido máximo
de **3.800 RPM**. Essa aceleração é um ponto inicial de teste, não medição.
Parada usa 0 RPM em VelocityVoltage para frear sem aguardar rampa de descida;
leitura inválida, desabilitação ou falha de controle aplicam saída neutra.

DutyCycleOut é saída aberta. O valor medido **0,028** foi registrado em
`BREAKAWAY_DUTY_CYCLE` e convertido para **kS = 0,336 V** usando 12 V nominais.
É uma aproximação inicial: confirmar tensão real e ajustar kS/kV/kP com o
mecanismo carregado. Não é um PID identificado. Os limites existentes de
40 A supply / 60 A stator permanecem ativos.

Leitura de velocidade com mais de 100 ms, valores não finitos, erro de
configuração, reset ou falha do pedido bloqueiam saída. MotorSafety corta
na ausência de novos pedidos por 100 ms. Depois de reset/configuração
perdida, reiniciar o programa com o robô desabilitado para reconfigurar.

A simulação acompanha o perfil de aceleração, mantém sua física e o kS
virtual anterior (0,20 V), pois o atrito real não foi identificado no modelo.
O modelo paralelo continua recebendo a tensão medida.

A sequência exige 100 ms de prontidão contínua antes de começar/retomar a
alimentação. Perda de mira ou sensor corta a centrífuga; a pequena tolerância
temporal fica restrita a oscilações de RPM com a mira válida. A regra anterior
de fallback por tempo das rodas ainda existe com seu piso de velocidade e
agora também exige sensores válidos; ela não equivale a RPM na tolerância.

Diagnóstico: `Centrifuga/Estado`, `AlvoRpm`, `Rpm`, `ErroRpm`, `ReferenciaRpm`
e `Centrifuga/Inputs/{ConfigurationApplied,ConfigurationStatus,ControlRequestOK,ControlStatus}`.
Se ficar parada com B, conferir também `Shooter/Sequencia/Estado`, que informa
se o bloqueio vem de pose, alcance, pivô ou rodas. Melhorar a rampa não resolve
por si só uma falha física de CAN/alimentação.

Referências: [CTRE Motion Magic Velocity](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/motion-magic.html#motion-magic-velocity),
[CTRE malha fechada e unidades dos ganhos](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/closed-loop-requests.html),
[WPILib Commands e botões](https://docs.wpilib.org/en/stable/docs/software/commandbased/binding-commands-to-triggers.html).

## Verificação

`gradlew check --offline --console=plain --continue`: **BUILD SUCCESSFUL**,
204 testes, zero falhas e zero erros. Inclui ciclo completo de coleta/tiro,
logs, replay, simulação dos elevadores, intake, perfil da centrífuga,
prioridade dos botões, cancelamento e perda/NaN de sensores do shooter.
Os testes executam no computador. Não houve deploy nem validação com carga
no robô físico nesta alteração.
