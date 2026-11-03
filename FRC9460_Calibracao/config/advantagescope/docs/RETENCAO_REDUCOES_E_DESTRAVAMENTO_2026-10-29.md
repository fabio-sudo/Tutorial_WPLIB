# Intake, reduções e preparação da centrífuga — 29/10/2026

## Dados confirmados pela equipe

- Intake CAN 34/35: dois estágios 9:1 por motor, total 81:1.
- Pivô da mira CAN 32: redução TOTAL 9:1. Não é a redução das rodas de disparo.
- Centrífuga CAN 36: redução total 5:1.
- Testes de partida foram feitos com motor e redutor juntos. Os percentuais observados não são multiplicados pela redução.

## Ruído da intake e registro novo

Fonte: `C:/Users/Admin/Documents/tuner-plot.csv`, 38.716 linhas, tempo 307,365–503,117 s.
SHA256: `221d156f2d578abccf20ae7081ec3f7cc7f41aeaf8d4fa969596c9bf495a0c21`.
Este arquivo contém apenas o motor esquerdo CAN 34, não o direito.

Entre 456 e 460 s: velocidade zero, posição -0,27002 volta, tensão 0,27 V e corrente de estator mediana 1,24 A. Compatível com o PID proporcional de retenção tentando corrigir um pequeno erro. O arquivo não contém o alvo nem o modo de controle, portanto não prova sozinho qual comando estava ativo.

Também houve retorno rápido próximo de 454 s: pico de 9,63 V e 42,52 A de estator; velocidade máxima desse episódio 83,48 rot/s. O pico global de velocidade foi 84,51 rot/s perto de 433,76 s. Não é apenas um ruído estacionário. A retenção antiga iniciada com alvo zero pode explicar retornos após movimento pelo Tuner, mas isso permanece hipótese sem o registro dos pedidos de controle.

Alterações:

- Sem pedido de retenção no construtor do IO; ao habilitar, captura a posição atual com encoder válido. Não zera o encoder novamente nesse momento.
- Desabilitado ou sensor inválido: neutraliza e descarta o alvo de retenção anterior.
- Ao soltar o movimento, preserva um alvo fixo. Chamadas repetidas de parada não arrastam esse alvo.
- **Substituído em 30/10/2026.** O PID relaxava para neutro quando erro ≤0,10 volta e velocidade ≤0,25 rot/s, e retomava com erro ≥0,20 ou velocidade acima do limiar. Isso causava o barulho da intake parada: o P puro (kP=1, sem kG) precisa de um erro estável de 0,03 a 0,08 volta para segurar o peso, que cai dentro da faixa de relaxar. O neutro tirava a força, o braço cedia, o PID retomava e o ciclo se repetia 20 a 40 vezes por segundo (325 trocas em 20 s na simulação).
- **Agora o modo depende só de onde o braço está parado, sem histerese:** no batente aberto (alvo até `BRAKE_ONLY_NEAR_OPEN_ROTATIONS` = 0,25 volta do limite aberto) só Brake, porque a gravidade já encosta o braço no batente (tensão 0 V); no meio do curso e no fechado, PID contínuo. Só Brake nessas posições deixa o braço derrapar na simulação (fechado 7° em 30 s e queda total em ~150 s; meio do curso 47° em 30 s).
- Consequência: aberto, o braço assenta na margem de segurança, uns 2,2° abaixo do limite operacional (`OPEN_SAFETY_MARGIN_ROTATIONS`), e `isOpen()` conta isso como aberto.
- Retenção PositionVoltage limitada a ±2 V no controlador. A saída manual DutyCycleOut permanece ±15%; ±3% ficou registrado como observação de partida, não como novo comando ou ganho.
- Ambos os motores continuam em Brake; seguidor oposto ao líder. Publicação DutyCycle e MotorVoltage em 50 Hz atende aos dois modos usados pelo seguidor.
- Logs: `Intake/RetencaoEmRepouso` (só Brake no batente aberto), `CorrecaoPosicaoAtiva` (PID segurando), `ErroRetencaoVoltas`.

Brake não garante sustentação estática de carga: por isso o PID segura contínuo fora do batente aberto. Com o PID contínuo a tensão de segurar é estável e de poucas dezenas de mV (simulação); se ainda houver chiado com a tensão constante, é a saída de duty muito baixo, não alternância. Validar queda, aquecimento e corrente com o mecanismo apoiado e movimento curto antes de operar o curso todo. Continua necessário ligar/reiniciar o programa com a intake fechada, pois o zero inicial é manual.

**Atualizado (29/10/2026, depois):** reiniciar o programa (deploy) não zera mais; só o Talon recém-ligado é zerado. Basta ligar o robô com a intake fechada. Zero manual: Back + Y do operador, desabilitado (`frc.robot.util.TalonZeroMark`).

## Pivô CAN 32

Antes: 24:1 inferidos de 6 voltas = 90°. Agora: 9:1 confirmados pela equipe, 40° por volta do motor, limites ±2,25 voltas = ±90°.

**Substituído depois pela medição no Tuner** (`MEDICAO_CURSO_PIVO_2026-10-29.md`): 132° em 8,98 voltas do rotor (≈14,699°/volta), limites ±130° = ±8,8439 voltas. Os valores 9:1 e ±2,25 acima não valem mais.
Mantidos os limites angulares existentes; corrigidos os soft limits, as conversões, o modelo físico e a telemetria. Perfil preserva 75°/s, 150°/s² e 900°/s³. Tolerância de 0,75° e limiar de atualização de 1,2° foram convertidos para a nova escala. Ganhos PID não foram aumentados automaticamente. Novo log: `Shooter/Pivo/ReducaoTotal`.

O zero continua exigindo shooter centralizado ao ligar. Confirmar sentido e ângulo com um movimento pequeno, antes de testar os extremos da fiação.

## Centrífuga e botão B

O controle e as curvas existentes continuam em RPM do rotor, com feedback 1:1. A redução 5:1 é usada para apresentar RPM/voltas da saída e para medir o destravamento. Exemplo: 2.300 RPM de rotor = 460 RPM na saída. O modelo físico foi atualizado para 5:1 mantendo o contrato de IO em unidades do rotor.

Nova preparação no botão B do piloto, por composição de Commands com requisitos dos subsistemas:

1. Para as rodas do shooter e freia a centrífuga; confirma rotor abaixo de 15 RPM por 0,10 s.
2. Reverte a 150 RPM do rotor (30 RPM na saída), até medir 0,25 volta de saída = 1,25 volta do rotor.
3. Freia novamente, confirma parada e então inicia a sequência existente de mirar, preparar RPM e alimentar.

Os 0,25 de volta são o ponto de início da frenagem, não uma promessa de parada sem ultrapassagem. São valores iniciais conservadores a verificar com bolinhas; não foram caracterizados mecanicamente.

Proteções: timeout de reversão 1,5 s; timeout de frenagem 0,75 s; corrente ≥15 A por 0,10 s bloqueia; sentido oposto ao esperado em mais de 0,03 volta de saída bloqueia; sensor inválido bloqueia. Os limites físicos de corrente do Talon permanecem 40 A de alimentação / 60 A de estator; o limiar de 15 A é uma interrupção supervisionada pelo código, não um limitador instantâneo de corrente.

Falha impede avançar ao tiro e não repete sozinha; é necessário soltar o botão.

**Mudado depois do teste no robô (29/10/2026):** a reversão fraca falhava sempre e o B nunca chegava ao tiro. Agora corrente alta, tempo esgotado, não parou e sensor oscilando só pulam o destravamento (aviso na Driver Station) e seguem para o tiro normal, que tem as próprias travas. Só `SENTIDO_INCORRETO` ainda bloqueia. Novo log: `Destravar/BloqueiaTiro`.

**Depois, no mesmo dia:** a reversão saiu do B (a centrífuga só gira para a frente). O `CentrifugaDestravarCommand` e seus testes ficam no projeto, sem botão. Soltar B ou desabilitar cancela e para. A preparação acontece uma vez por acionamento, não em cada oscilação da prontidão de RPM. Os comandos isolados de teste e as chamadas diretas anteriores de tiro permanecem sem essa preparação; o novo comportamento está no B do piloto.

Logs: `Centrifuga/ReducaoTotal`, `SaidaRpm`, `SaidaVoltas`, `Destravar/Estado`, `Destravar/Falha`, `Destravar/VoltasSaida`.

## Validação

Verificação local `gradlew check --offline --continue`: compilação, testes de limites/retomada da intake, conversão do pivô, reversão/frenagem física simulada, cancelamento, corrente alta, timeout, sensor inválido e sentido incorreto. Integração de coleta e tiro, visão, logs e replay também executadas. Nenhum deploy ou movimento físico realizado nesta alteração.

## Documentação usada

- [CTRE VoltageConfigs 26.3.0](https://api.ctr-electronics.com/phoenix6/stable/java/com/ctre/phoenix6/configs/VoltageConfigs.html): limite de tensão em controles de tensão.
- [CTRE Follower 26.3.0](https://api.ctr-electronics.com/phoenix6/stable/java/com/ctre/phoenix6/controls/Follower.html): alinhamento oposto e frequência dos sinais de saída.
- [CTRE controle em malha fechada](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/closed-loop-requests.html): unidades de ganhos e compensação de gravidade.
- [WPILib triggers](https://docs.wpilib.org/en/stable/docs/software/commandbased/binding-commands-to-triggers.html): agendamento e cancelamento por botão.
