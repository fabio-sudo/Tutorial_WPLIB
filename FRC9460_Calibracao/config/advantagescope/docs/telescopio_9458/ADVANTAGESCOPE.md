# Telescópio 9458 no AdvantageScope

O modelo está em `../../mecanismos/Robot_Telescopio_9458` (FRC9458 -
Telescopio): `model.glb` é o tubo 4" fixo com o acionamento, `model_0.glb` o
tubo 3" e `model_1.glb` o tubo 2". O motor fica na base (embaixo) e os tubos
saem para cima pela ponta oposta.

1. Importe `../../layouts/FRC9460-Simulacao3D.json` em **File > Import Layout**.
2. Abra `../../mecanismos/Robot_Telescopio_9458/Telescopio_9458-Demo.wpilog`
   em **File > Open Log(s)**.
3. **Telescopio 9458** mostra o desenho lateral (aba Mechanism, 2D).
4. **Telescopio 9458 3D** mostra os GLBs articulados.

## Simulação Java (testar com o controle)

O código fica em `src/main/java/frc/robot/subsystems/telescopio_9458`, com a mesma
estrutura e as mesmas proteções do telescópio 9460. Ele roda junto com o
robô e o 9460; o telescópio fica em bancada, com a base na origem.

1. No VS Code, **WPILib: Simulate Robot Code** com a **Simulation GUI**.
2. Na Simulation GUI, arraste o controle de **System Joysticks** para **Joysticks [0]**.
3. No AdvantageScope, **File > Connect to Simulator > NetworkTables 4 (AdvantageKit)**.
4. Importe `../../layouts/Telescopio9458-Simulacao.json`.
5. Coloque em **Teleoperated** e **Enabled** e use o painel **Tuning > Telescopio9458**.

> **Desde 28/10/2026 o 9458 está fora de uso:** o subsistema, o painel e o SysId dele estão **comentados** no `RobotContainer` (cabeçalho "ANTIGO"), então nada dele roda no robô 9460. Os botões (RT/LT no Teleop e R1/L1 no modo Test) também. Para voltar, descomente os blocos e os imports; a tabela abaixo é o que eles fazem.

| Controle (segurar) | Ação |
|---|---|
| **RT** | Abre até 0,94568 m na ponta, 5 cm antes do batente (74,463 voltas do motor). |
| **LT** | Fecha até o batente. |
| Soltar | O perfil freia e mantém a altura contra a gravidade. |

Os gatilhos só valem sem BACK nem START (BACK + LT/RT é SysId e START + LT/RT é a
calibração do steer); os dois juntos não movem.
Hoje o controle é pelo painel: **Tuning > Telescopio9458** (`MetaMetros`,
`ControleAtivo`, `CicloAutomatico`), como no 9460.

| Aba do layout | O que mostra |
|---|---|
| Telescopio 9458 3D | GLBs: tubo 3" sobe metade e tubo 2" a ponta inteira. |
| Telescopio 9458 2D | Desenho lateral (Mechanism). |
| Controle | Gatilhos apertados. |
| Posicao | Meta, medida e perfil; faixas `Comando` e `Estado`. |
| Estagios - cascata | Curso de cada estágio, da ponta e voltas do motor (74,463 na meta superior). |
| Motor | Tensão, corrente (limite de 30 A) e faixas de limite de corrente, travamento e topo. |
| Velocidade da ponta | Até 0,55 m/s no trecho central; aproximação dos extremos a 0,06 m/s. |

Proteções: limite de 30 A no estator (simulado), travamento a partir de 22 A sem
movimento por 0,25 s, topo só com kG e recolhido em 0 V.

## Robô físico (TalonFX ID 5)

O motor do telescópio 9458 físico é um TalonFX (Kraken X60) no **ID 5**, no barramento
do roboRIO (o swerve usa 10 a 21 e o Pigeon usa 3). `TelescopioIOTalonFX_9458` aplica
os limites de 30 A, os soft limits (75,2504 voltas para cima e zero para baixo) e o modo Brake, e **zera a
posição ao ligar**: o telescópio precisa estar fechado quando o robô liga. Se a
configuração ou o zero falharem, a posição fica inválida e o telescópio não se move.

Primeiro teste, com o telescópio fechado e o robô recém-ligado:

1. No Phoenix Tuner X, confirme o TalonFX no ID 5.
2. Conecte o AdvantageScope ao robô e importe `../../layouts/Telescopio9458-Simulacao.json`
   (as chaves são as mesmas no robô real; `Telescopio9458/Modo` mostra `ROBO_REAL_TALONFX_ID_5`).
3. Em Teleoperated, dê um toque curto no **R2**: o telescópio deve **abrir**.
   Se fechar, troque `MOTOR_INVERTED` para `true` em `TelescopioConstants_9458`.
4. Abra 10 cm por estágio e compare: `MotorRotacoes` deve marcar cerca de 15,75 voltas.
5. Após validar o movimento em posições intermediárias, a meta máxima de operação
   é 74,463 voltas e 0,94568 m na ponta. O batente físico continua em 78,4 voltas.

### Margem, aproximação e comparação em centímetros

A margem de abertura escolhida pela equipe é **5 cm na ponta** (2,5 cm por
estágio). A meta máxima é **94,568 cm**, equivalentes a **74,463 voltas do motor**
ou **4,654 voltas na saída do redutor**. A proteção adicional do Talon corta a
abertura em **95,568 cm**, ainda 4 cm antes do batente físico de **99,568 cm**.
Essa reserva depende do zero e da geometria corretos; não é garantia de frenagem.

No recolhimento, a meta permanece zero. O perfil se aproxima dos extremos a
**6 cm/s nos últimos 2 cm**, com aceleração de **1 m/s²**. A velocidade central foi
ajustada para **55 cm/s**, conforme pedido da equipe. Soltar em velocidade máxima
permite aproximadamente **15,1 cm** de frenagem do perfil; não é parada instantânea.
Metas numericamente equivalentes a zero são normalizadas para evitar tensão
de sustentação no batente por arredondamento.

Reimporte `../../layouts/Telescopio9458-Simulacao.json`. As abas adicionais são:

- **Saida da ponta - fisico e simulado (cm e m)**: extensão dos dois, em centímetros
  e metros, e diferença físico menos simulado. Zero significa recolhido; não é altura
  absoluta em relação ao chão. O físico é calculado pelo encoder e transmissão.
- **Saida da ponta - comparacao e limites (cm)**: gráfico com as duas extensões,
  meta máxima, corte Talon e batente físico.
- **Limites - margem de seguranca na ponta**: distância restante ao batente superior,
  margem e configuração de movimento efetivamente publicada pelo programa.

Confira `Controle/Versao = APROXIMACAO_CURTA_ZERO_TOLERANTE_3`, velocidade `0.55`,
aceleração `1.0` e aproximação `0.06` em `/RealOutputs/Telescopio9458` depois de
enviar o programa. A análise do ensaio anterior está em
[ANALISE_CSV_2026-10-22.md](ANALISE_CSV_2026-10-22.md).

**Ajuste posterior da chegada:** o novo CSV mostrou aproximadamente 0,62 s nos
últimos 3 cm. A zona lenta foi reduzida de 3 para 2 cm, mantendo 6 cm/s na
aproximação e o PID. Nos testes com ganhos reais e atraso de leitura de 20/40 ms,
o trecho final passou de 0,56–0,66 s para 0,42–0,48 s, com velocidade a 1 cm do
limite inferior/superior abaixo de 0,10 m/s. Validar no mecanismo físico após deploy.

Os ganhos do hardware ficam em `REAL_KP`, `REAL_KI`, `REAL_KD`, `REAL_KS`,
`REAL_KG`, `REAL_KV` e `REAL_KA`, em `TelescopioConstants_9458.java`. Os valores
iniciais preservam os anteriores, ainda sem caracterização física. Alterá-los não
altera a massa ou os parâmetros da simulação.

## Real e simulado ao mesmo tempo

O AdvantageScope conecta a **uma** fonte ao vivo por vez: **File > Connect to Robot**
(na bancada 9458: `10.94.58.2`) ou **File > Connect to Simulator** (`127.0.0.1`). O layout
`Telescopio9458-Simulacao.json` serve para os dois, porque as chaves são as mesmas.

Para ver os dois juntos ao vivo, o `Telescopio_9458` roda um **modelo em paralelo**: a
mesma física da simulação, recebendo a tensão **medida na saída do TalonFX**.
Ela já incorpora os efeitos dos limites do controlador; o modelo não aplica um
segundo limite de corrente a essa tensão. Conectado só ao robô, o layout abre
diretamente a aba com **dois telescópios 3D**:

| Aba | O que comparar |
|---|---|
| Fisico x virtual 3D - encoder e previsao | Físico na origem, pelo encoder; virtual 0,6 m ao lado, pela previsão física. A orientação na tela depende da câmera. |
| Voltas - motor e saida do redutor | Tabela numérica com voltas do motor, redução, voltas da saída e comparação com o virtual. |
| Fisico x virtual - voltas na saida do redutor | Gráfico das voltas após a redução, físico em azul e virtual em laranja, com a diferença em vermelho. |
| Fisico x virtual - posicao e erro | Extensão física (azul), virtual (laranja), meta e diferença físico menos virtual (vermelho). |
| Fisico x virtual - corrente e velocidade | Corrente e velocidade do motor real contra o modelo. |
| Fisico x virtual - tensao e validade | Tensão solicitada, tensão medida e tensão usada pela previsão; validade da comparação. |
| Diagnostico - encoder e conexao | Voltas do encoder sem limitar, extensão, comunicação, referência, reset e códigos de retorno do Talon. |
| PID - erro e parcelas da tensao | Erro em relação ao perfil, contribuição PID e compensação de gravidade/atrito/velocidade/aceleração. |

- Com o robô desabilitado, o modelo é alinhado à posição medida sem avançar a
  física. `Modelo/ComparacaoAtiva` indica quando a comparação está em andamento.
- Se a leitura ficar inválida, a previsão pausa e `Modelo/Valido` fica falso.
  Ao recuperar a leitura, ela é alinhada novamente antes de continuar.
- Na simulação, real e modelo são iguais (diferença 0), porque é a mesma física.
- No robô, a diferença permite investigar massa, atrito, transmissão e referência.
  Ela não identifica sozinha a causa do desvio. A previsão ainda considera 5 kg
  e a transmissão configurada. Se o motor estiver desacoplado do telescópio,
  o desenho físico mostra uma extensão equivalente calculada pelas voltas;
  a diferença em relação ao modelo com carga é esperada.

Para usar na bancada: importe novamente `../../layouts/Telescopio9458-Simulacao.json`
em **File > Import Layout**. Em **App > Show Preferences**, use `Robot Address`
`10.94.58.2` para o roboRIO 9458 na rede padrão. Conecte em **File > Connect to Robot >
NetworkTables 4 (AdvantageKit)** e mantenha a linha do tempo no instante atual.
Por USB, selecione **File > Use USB roboRIO Address**. Os dois modelos são publicados
pelo programa do roboRIO; não é necessário executar o simulador no computador.

Para comparar depois, abra os dois logs juntos em **File > Open Log(s)...**: o do
robô (baixado do roboRIO) e o da simulação (`logs/simulacao/Simulacao_*.wpilog`).
As abas **Telescopio 9458** e **Telescopio 9458 3D** do layout completo também
funcionam com a simulação, porque as chaves são as mesmas do log de demonstração.

## Ajustar PID e compensações durante os testes

### Ler as voltas com a redução

Na aba **Voltas - motor e saida do redutor**, os campos de `RealOutputs/Telescopio9458/Voltas`
mostram `Motor` (encoder), `Reducao` (16) e `SaidaRedutor` (motor dividido por 16).
Em `Modelo/VoltasSaidaRedutor` aparece a previsão virtual da mesma saída.
Os valores são posições em voltas desde o zero: aumentam ao abrir e diminuem ao fechar,
sem limitar a leitura física ao curso do desenho. Não representam a soma de todas
as voltas percorridas nos dois sentidos.

**Redução 16:1: 16 voltas do motor = 1 volta da saída; 78,4 voltas = 4,9 voltas da saída.**
Uma volta da saída move a corrente 0,1016 m e a ponta da cascata 0,2032 m.
A redução já é aplicada pelo código a esses campos; não dividir novamente no AdvantageScope.

### Aplicar os ganhos

O controlador executa no **roboRIO**, usando **metros da ponta**, e envia tensão
ao TalonFX. Ganhos calculados em voltas do rotor ou para um Slot do Phoenix têm
outras unidades e não devem ser copiados diretamente.

1. Desabilite o robô.
2. Em **Tuning > Telescopio9458 > PID**, preencha `kP`, `kI`, `kD`, `kS`, `kG`,
   `kV` e `kA`. Mudar os números apenas prepara o ajuste.
3. Coloque `AplicarGanhos` em `true`. O pedido é consumido automaticamente.
4. Confira `RealOutputs/Telescopio9458/Ajuste/Resultado = GANHOS_APLICADOS`.
5. Habilite e use RT/LT ou uma meta no painel. Compare posição, perfil e tensão.
6. Ao concluir a calibração, copie os valores aprovados para as constantes `REAL_*`
   e faça um novo deploy. Os ajustes do painel não substituem esses valores de partida.

Pedidos com o robô habilitado são recusados e não ficam pendentes para depois.
Valores negativos ou não finitos são recusados; `kS` e `kG` não podem exceder 6 V.
O conjunto é aplicado de uma vez. A contribuição integral é limitada a ±1 V e
a tensão total continua limitada a ±6 V. Os ganhos usados são gravados em
`Telescopio9458/Ganhos` para reprodução no replay.

| Ganho | Unidade / finalidade |
|---|---|
| kP | V/m de erro de posição |
| kI | V/(m·s), integral do erro |
| kD | V/(m/s), derivada do erro |
| kS | V, compensação de atrito no sentido do movimento |
| kG | V, compensação de gravidade |
| kV | V/(m/s), velocidade da ponta |
| kA | V/(m/s²), aceleração da ponta |

## Recuperar a referência depois de uma falha

A partida continua zerando o encoder com o mecanismo recolhido, conforme o
procedimento da equipe. Um reset posterior do Talon invalida a referência e
bloqueia comandos de movimento. Reconexão após falha da configuração inicial
também exige uma confirmação; a posição não é zerada silenciosamente no meio do curso.

1. Desabilite o robô e coloque fisicamente o mecanismo na posição recolhida.
2. Em **Tuning > Telescopio9458**, coloque `ConfirmarRecolhidoEZerar` em `true`.
3. Confira `Referencia/Resultado = ZERO_CONFIRMADO`, `Inputs/PositionValid = true`
   e `Diagnostico = OK`. O código reaplica configuração, limites e zero.

Esse comando **não recolhe o telescópio automaticamente**. A confirmação do operador
substitui temporariamente o sensor de referência que será instalado depois.

## Parâmetros

A redução e o fim de curso vêm do robô físico; a coroa e a altura fechado vêm
do CAD ([comparação CAD x medidas](telescopio_9458_cad_x_medidas.png)).
São os mesmos valores de `TelescopioConstants_9458` (Java).

| Parâmetro | Valor | Origem |
|---|---|---|
| Altura fechado | 0,780 m (topo do tubo 2"; a equipe mediu 0,80 m) | CAD |
| Fim de curso | **78,4 voltas** do rotor (antes da redução) | robô |
| Montagem | cascata: os dois estágios sobem juntos, 10 cm + 10 cm | equipe |
| Motor e redução | Kraken X60, duas reduções de 4:1 = 16:1 | robô |
| Coroa | REV-21-2016: 16 dentes, corrente #25 (passo 6,35 mm), primitivo 32,55 mm | CAD |
| Curso por estágio | 0,498 m (o CAD tem ~0,57 m de folga mecânica) | calculado |
| Deslocamento da ponta | 0,996 m (altura aberta 1,776 m; medida: 1,75 m) | calculado |
| Massa | 5 kg (para a simulação física) | equipe |

O redutor desenhado no CAD é 4:1 × 3:1 = 12:1; ele não foi usado, porque o
robô físico tem 4:1 × 4:1.

Pela transmissão, a corrente anda 16 × 6,35 mm = 101,6 mm por volta da coroa.
Cada volta do motor move cada estágio 101,6 mm ÷ 16 = 6,35 mm, e a ponta o dobro
(12,7 mm). Com 78,4 voltas, cada estágio sobe 0,498 m e a ponta 0,996 m:
0,780 m + 0,996 m ≈ 1,776 m aberto, 2,6 cm acima dos 1,75 m medidos.
Os valores ficam no início de `../../scripts/gerar_log_demo_componentes.py`.

## O que o log mostra

O log tem 15 s; os dois estágios sempre andam juntos:

| Tempo | Movimento | Ponta | Motor |
|---|---|---|---|
| 0–1,5 s | fechado | 0 | 0 voltas |
| 1,5–4,5 s | cada estágio sobe 10 cm | 0,2 m | 15,75 voltas |
| 6–9 s | sobe até o fim de curso | 0,996 m | 78,4 voltas |
| 10,5–13,5 s | fecha | 0 | 0 voltas |

Os tempos são de demonstração, mais lentos que o real para facilitar a
visualização. Na velocidade livre do Kraken X60 (~6000 rpm), as 78,4 voltas
levariam cerca de 0,8 s, sem considerar aceleração nem limite de corrente.

No desenho, azul representa a base fixa, laranja o tubo 3" e verde o tubo 2".
Os comprimentos e as posições recolhidas vêm das caixas geométricas dos
grupos do CAD; o desenho é esquemático e omite correntes, fixadores e
detalhes do acionamento.

O log de demonstração não tem física de motor, gravidade, corrente nem controle;
isso está na simulação Java acima. O modelo GLB, sozinho, não publica posições.

Dados publicados pelo log de demonstração:

- `/RealOutputs/Telescopio9458/Pose`: base do telescópio (na origem).
- `/RealOutputs/Telescopio9458/Componentes`: poses dos dois tubos no modelo 3D.
- `/RealOutputs/Telescopio9458/ExtensaoTubo3Metros` e `ExtensaoTubo2Metros`:
  extensão de cada estágio; `ExtensaoMetros`: deslocamento da ponta.
- `/RealOutputs/Telescopio9458/MotorRotacoes`: voltas do motor equivalentes.
- `/RealOutputs/Telescopio9458/AlturaTotalMetros`: 0,780 m + deslocamento.
- `/RealOutputs/Telescopio9458/Mecanismo2d`: desenho da aba Mechanism.
- `/RealOutputs/Telescopio9458/Modo`: `DEMONSTRACAO_VISUAL_SEM_FISICA`.
- `/RealOutputs/Telescopio9458/CursoRealConfirmado`: verdadeiro.
- Constantes: `CursoPorEstagioMetros`, `MotorRotacoesNoLimite`,
  `ReducaoTotal`, `DentesCoroa`, `PassoCorrenteMetros`,
  `DiametroPrimitivoCoroaMetros`, `MassaKg`.

`../../scripts/gerar_log_demo_componentes.py` regenera o log e
`../../scripts/preparar_simulacao_3d.py` prepara as abas do layout.
