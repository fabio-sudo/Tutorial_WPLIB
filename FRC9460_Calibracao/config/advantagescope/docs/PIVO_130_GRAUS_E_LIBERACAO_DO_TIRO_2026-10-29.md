# Pivô ±130°, velocidade da mira e liberação do tiro

Atualização de 29/10/2026. Substitui as hipóteses anteriores de 9:1 como redução total, ±90° e proposta de ±9 voltas. O redutor informado continua registrado como 9:1, mas a relação efetiva até a mira é calibrada pelo conjunto montado.

## Medição e zero

Fonte: `C:/Users/Admin/Desktop/logs/tuner-plot.csv` e print fornecido pela equipe; 17.482 linhas, 183,711–395,741 s.
SHA256: `cdbff2c7e6cff7cfce60ab7af989ec420d902a18286e1f3c76ddef3f1aa4e615`.

A equipe informou 132° para o patamar mostrado em +8,895020 voltas. O CSV começa com um patamar estável em -0,084961 volta. Foi considerado esse patamar inicial como o centro físico mencionado pela equipe, não como zero numérico perfeito. Se o centro físico no ensaio era outro ponto, corrigir a calibração antes de confiar nos graus exibidos.

- Deslocamento medido: 8,895020 - (-0,084961) = **8,979981 voltas do rotor**.
- Conversão: **14,69936295° por volta do rotor**.
- Redução efetiva calibrada: **24,49085727:1**; estimativa da medição, não contagem de dentes.
- Limites solicitados: **-130° e +130° relativos ao centro**, total 260°.
- Limites equivalentes após zerar no centro: **-8,84392068 e +8,84392068 voltas do rotor**.
- Margem até o ponto de referência de 132°: **2°**, ou **0,13606032 volta de rotor**.

O limite simétrico segue a solicitação explícita da equipe de aplicar o equivalente nos dois sentidos. Não foi usado o mínimo negativo de -10,565430 do CSV para ampliar o curso. O offset inicial foi descontado apenas na calibração: não é aplicado como deslocamento ao zero do robô. Rotação positiva/negativa não determina sozinha direita/esquerda sem conferir a montagem e a inversão.

Mira, limites do Talon, telemetria e modelo físico usam a mesma conversão. Feedback do Talon permanece 1:1 em voltas do rotor. A simulação usa ±132° como envelope de referência, sem pretender modelar a elasticidade da fiação. A margem de 2° não é garantia de parada em todas as cargas e velocidades.

## Mira mais rápida

Pedido da equipe: alinhar mais rápido. Perfil Motion Magic alterado de 75°/s e 150°/s² para:

- Velocidade máxima: **100°/s**.
- Aceleração: **200°/s²**.
- Jerk: **1.200°/s³**.

Os valores são convertidos para rotações do motor nas constantes. Os limites continuam ±130°. O modo manual usa 3% para permitir ajustes lentos; não é a velocidade usada pelo Motion Magic.

## Por que START girava a centrífuga e B não

Foram consultados os eventos físicos em `C:/Users/Public/Documents/FRC/Log Files/DSLogs/2026_10_29 14_20_11 qui.dsevents` e `2026_10_29 14_10_19 qui.dsevents`. Há registros explícitos de `PIVO_BUSCANDO_ALVO`, com erro de 1,7–2,9° em várias tentativas e top/tower prontos; a tolerância da mira é 0,75°. Há também tentativas com tower abaixo do RPM e uma com alvo exigindo 167°, fora do curso. Logo, nem toda tentativa tem a mesma condição.

O B estava sendo bloqueado pela mira nas tentativas citadas. START usa saída direta, por isso funcionar no START não prova que todas as condições de tiro estavam satisfeitas. O código atualmente encontrado no projeto liga o B diretamente à sequência de tiro, sem a reversão inicial; essa configuração foi preservada.

Com kP = 2 V/volta, um erro de 2° na escala calibrada produz aproximadamente 0,272 V. Isso é compatível com faltar esforço para vencer o atrito observado próximo de 3% no ensaio. É uma hipótese sustentada pelos dados, não diagnóstico mecânico conclusivo.

Foi acrescentado **kS inicial de 0,30 V**, um pouco abaixo dos 0,36 V nominais que 3% representariam a 12 V. O sinal segue o erro de posição (`UseClosedLoopSign`), permitindo corrigir erro residual depois que a referência de velocidade chega a zero. Mantidos kP/kI/kD e tolerância de 0,75°; não foi liberado tiro com mira desalinhada. Este ganho precisa de validação física: ainda pode ser insuficiente em algumas posições ou produzir oscilação perto do alvo.

O simulador atual não modela atrito seco medido. Não recebeu o kS físico isoladamente, para evitar inserir uma força sem o atrito correspondente. Testes de simulação verificam geometria, limites e sequência, não certificam a calibração física de atrito.

## Controle de operador opcional

O texto anexado continha avisos de botão 6 indisponível na porta 1. O controle de operador agora verifica presença e quantidade de botões/eixos antes da leitura; funciona se conectado depois e cancela comandos ao perder o botão disponível. Os avisos gerais da WPILib não foram desativados.

A perda de comunicação com o robô também aparece nos eventos, junto de atrasos de ciclo. Os dados consultados não comprovam que o aviso de joystick causou essa perda; esta alteração não é apresentada como solução da rede.

## Validação e documentação

Verificação local completa passou: compilação, conversão e alcance angular, limites nos dois sentidos, simulação, coleta/tiro, controles opcionais, visão e replay. Nenhum deploy ou comando físico foi realizado nesta conversa. Após enviar a versão, começar conferindo centro/sentido em deslocamento curto, depois validar aproximação dos limites e erro de mira sob carga.

- [CTRE — soft limits](https://api.ctr-electronics.com/phoenix6/stable/java/com/ctre/phoenix6/configs/SoftwareLimitSwitchConfigs.html)
- [CTRE — sinal de kS em controle de posição](https://api.ctr-electronics.com/phoenix6/stable/java/com/ctre/phoenix6/signals/StaticFeedforwardSignValue.html): recomenda valor mínimo para evitar oscilação perto do zero.
- [WPILib — CommandXboxController](https://github.wpilib.org/allwpilib/docs/release/java/src-html/edu/wpi/first/wpilibj2/command/button/CommandXboxController.html)
