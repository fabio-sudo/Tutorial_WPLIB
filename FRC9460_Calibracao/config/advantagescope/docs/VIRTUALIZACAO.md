# Virtualização do Swerve e das câmeras

## Etapa implementada

O robô virtual começa em **X = 2 m, Y = 2 m, orientação = 0°**.
O Drive atualiza uma segunda `SwerveDriveOdometry` usando os encoders e o
giroscópio simulados. Ela movimenta as câmeras PhotonLib e não recebe correções
da visão. O estimador normal do Drive continua separado.

Essa referência é cinemática: compartilha os sensores virtuais com o estimador.
Não modela derrapagem, contato com obstáculos ou colisões. Portanto, erro quase
zero entre as duas odometrias não comprova a precisão do robô físico.

**Fusão PhotonLib ligada.** Cada câmera tem seu próprio `PhotonPoseEstimator`
e `VisionIOSim`. O cenário entrega as estimativas em lotes ao IO; depois de
`Logger.processInputs`, o `Vision` valida cada observação e utiliza sua fila
temporal para enviá-la ao estimador do Drive. Um lote vazio nunca reaplica a
pose dos campos individuais. As antigas medições artificiais fixas foram removidas.

O sinal `Vision/Simulation/FusionEnabled` deve estar `true`. Com tags visíveis,
os contadores `Vision/Front/Fusion/AcceptedCount` e
`Vision/Rear/Fusion/AcceptedCount` crescem conforme cada câmera consegue medir.
Uma câmera sem alvos ou desconectada não impede o funcionamento da outra.
O projeto funde X/Y; a orientação do Drive continua vindo do giroscópio.

O robô físico mantém seus IOs Limelight. O cenário não é criado durante replay.
A simulação também grava WPILOG em `logs/simulacao`; a seleção explícita de REPLAY lê esses
inputs e recalcula a fusão. Veja [Gravação e replay da visão](REPLAY_VISAO.md).

## Como executar

1. Execute **WPILib: Simulate Robot Code**, com a Simulation GUI.
2. Conecte o AdvantageScope ao simulador em `127.0.0.1` por NT4/AdvantageKit.
3. Inicialmente em Disabled, confira a referência em `(2, 2, 0°)`.
4. Na Simulation GUI, confira o controle Xbox no índice esperado pelo projeto.
5. Ative Teleoperated e Enabled. Movimente devagar o controle e depois gire.
6. Volte a Disabled ao terminar. A referência pode terminar de acompanhar a
   desaceleração dos motores simulados, mas não deve saltar para `(2, 2)`.

Se uma câmera não enxergar tags na pose inicial, isso pode ser geometricamente
correto. O teste não força duas detecções; o campo de visão decide os alvos.

## Campos no AdvantageScope

Os caminhos abaixo ficam dentro de `/RealOutputs/` na execução desktop normal.

| Campo | Utilização |
| --- | --- |
| `Drive/Simulation/ReferencePose` | Referência independente dos sensores simulados |
| `Vision/Simulation/ReferencePose` | Mesma referência enviada ao mundo PhotonLib |
| `Robot/Pose` | Estimativa do Drive |
| `Vision/Simulation/Front/CameraPose` | Posição/orientação da câmera frontal no campo |
| `Vision/Simulation/Rear/CameraPose` | Posição/orientação da câmera traseira no campo |
| `Vision/Simulation/PositionErrorMeters` | Diferença atual entre estimador e referência |

No Campo 2D, sobreponha a referência e `Robot/Pose` com cores diferentes.
No Campo 3D, adicione as duas `CameraPose` como eixos.

Para conferir a fusão, use os campos abaixo em `Vision/Front/Fusion/` e
`Vision/Rear/Fusion/`:

| Campo | Significado |
| --- | --- |
| `ObservationType`, `LastAppliedAlgorithm` | Origem da última observação e da última correção; na simulação, `PHOTONVISION` |
| `BatchSize`, `ReceivedCount` | Tamanho do lote atual e total de observações examinadas sem repetição de timestamp |
| `NewMeasurement`, `QueuedCount` | Observações que passaram pelos filtros e entraram na fila |
| `MeasurementAccepted`, `AcceptedCount` | Pulso e contador de aplicações no estimador do Drive |
| `FusionResult`, `RejectedCount` | Último resultado e total de rejeições de entrada/fusão |
| `Valid`, `Reject/*` | Disponibilidade e motivos de rejeição da última observação/estado da câmera |
| `XYStdDevMeters`, `PoseErrorMeters` | Desvio utilizado e diferença contra a estimativa histórica do Drive |
| `ValidatedPoses` | Observações 3D que passaram pelos filtros neste ciclo; ainda podem aguardar/reprovar a fusão |
| `RejectedPoses` | Poses recusadas pelos filtros de entrada neste ciclo |
| `AppliedPoses` | Poses 2D efetivamente aplicadas no Drive neste ciclo |

`NewMeasurement` e `MeasurementAccepted` são pulsos em ciclos diferentes: a fila
aguarda uma janela limitada para ordenar capturas próximas. `Valid` pode ficar
verdadeiro entre frames de uma câmera sem haver medição nova naquele
ciclo. Os arrays de poses ficam vazios quando o ciclo não contém aquele evento.
Os caminhos antigos `MegaTag2/*` foram mantidos como aliases de compatibilidade
com as abas existentes; esse nome não identifica o algoritmo da câmera virtual.

Cada câmera tem os diagnósticos abaixo em
`Vision/Simulation/Photon/Front/` e `Vision/Simulation/Photon/Rear/`:

| Campo | Significado |
| --- | --- |
| `TagCount`, `TagIds` | Tags detectadas no último frame |
| `TagPoses` | Poses das tags detectadas; use objeto Vision Target no Campo 3D |
| `HasNewFrame`, `ProcessedFrameCount` | Chegada e processamento dos frames |
| `HasEstimatedPose`, `UsedTagCount`, `UsedTagIds` | Resultado e tags efetivamente utilizadas no cálculo de pose |
| `EstimatedRobotPoses` | Array para visualizar como Ghost no Campo 3D |
| `EstimatedRobotPose` | Última pose singular; pode permanecer antiga sem detecção |
| `CaptureAgeSeconds` | Idade da captura recebida |
| `ReferenceAtCaptureAvailable` | Há histórico suficiente para comparar o erro? |
| `EstimationErrorMeters`, `RotationErrorDegrees` | Erro contra a referência no instante da captura |

Use os arrays `TagPoses` e `EstimatedRobotPoses` na visualização: ficam vazios
quando o novo frame perde os alvos ou não produz estimativa. Sem histórico ou
sem pose, os erros ficam `NaN`; não representam zero erro.

## Configuração e limites

- `VisionFieldConstants` escolhe o mapa. O mundo virtual, os simuladores das
  câmeras e os estimadores PhotonLib usam esse mesmo layout explicitamente.
- `VisionCameraTransforms` reutiliza as constantes de montagem já existentes.
  A interpretação atual é RIGHT positivo para a direita e PITCH positivo para
  cima. As medidas da frontal e os sinais devem ser conferidos na montagem real.
- O cenário usa **todas as tags do mapa**. O filtro dos IDs das Limelights reais
  ainda não é aplicado ao diagnóstico PhotonLib. Assim, não compare diretamente
  sua quantidade de tags com uma Limelight configurada para apenas o hub.
- `VisionSimulationConstants` reúne a configuração óptica das duas câmeras.
  O perfil Limelight 4 usa **1280×800, 60 FPS configurados e exposição 0,66 ms**,
  conforme o pipeline informado. O FOV nominal da lente original é **82°
  horizontal e 56,2° vertical**; não é uma calibração dos intrínsecos reais.
  O simulador usa centro óptico no centro da imagem e distorção zero.
- Latência média de 30 ms, ruído em pixels e área detectável mínima de 100 pixels
  continuam sendo aproximações de ensaio. Não são especificações da Limelight.
  Não foi inventado um alcance máximo: ele permanece sem corte de distância
  até haver uma medida real. FOV, orientação da face e área da tag na imagem
  já limitam a detecção. A frequência efetiva também depende do ciclo do simulador.
- O downscale 2 e o filtro de área 0,0010% foram registrados a partir do pipeline.
  Downscale é referência documental: PhotonLib não reproduz o detector U-Michigan,
  seu limiar de qualidade, ganho ou iluminação. Ele não reduz o FOV pela metade.
  Confira os valores usados em `Vision/Simulation/Front/Configuration/` e
  `Vision/Simulation/Rear/Configuration/`.
- `VisionConstants` reúne confiança, fila e filtros da fusão. A decisão de aceitar
  cada pose continua em `Vision`; o Drive recebe a observação validada e atualiza
  seu estimador. Separar constantes não alterou os pesos ou limiares existentes.
- Não há simulação de iluminação real, nem garantia de oclusão pelas estruturas
  desenhadas pelo AdvantageScope. O modelo 3D do campo é uma visualização.
- PhotonPoseEstimator multitag com fallback de menor ambiguidade não é MegaTag2.
  O fallback utiliza uma única tag mesmo quando existem outros alvos no frame.
- A confiança usa a contagem e a distância média apenas das tags utilizadas.
  Mantivemos os desvios iniciais do projeto, que ainda precisam ser ajustados
  com dados reais. Capturas próximas de duas câmeras recebem redução conservadora
  de peso; contradições grandes rejeitam aquele grupo sem interromper a odometria.
- Os filtros PhotonLib rejeitam pose não finita, ausência de tags, distância
  inválida, captura antiga/futura, giroscópio indisponível/giro excessivo,
  posição fora do campo, altura acima de 0,30 m em módulo, roll/pitch acima de
  20° em módulo e ambiguidade de uma tag indisponível/negativa/acima de 0,20.
  Esses limiares são parâmetros iniciais do projeto, não limites oficiais.
- O timestamp PhotonLib já informa a captura na época FPGA do robô. A latência
  não é descontada novamente. As observações passam pelo mesmo histórico de
  odometria, deduplicação e ordenação usados pela integração Limelight.

## Verificação no AdvantageScope

1. Inicie uma nova execução da simulação após recompilar o código.
2. Confira `Vision/Simulation/FusionEnabled = true`.
3. Observe os `TagIds` e `HasEstimatedPose` de cada câmera. Com geometria favorável,
   seu `Fusion/AcceptedCount` deve crescer e `LastAppliedAlgorithm` ser `PHOTONVISION`.
4. Movimente devagar e gire. Compare `Robot/Pose` com a referência; acompanhe
   `FusionResult` e `Reject/*` quando uma câmera perder as tags.
5. Se apenas uma câmera enxergar tags, confira que o contador dela continua
   crescendo. Não é necessário que as duas enxerguem a mesma tag.
6. Em Disabled e parado, pequenas diferenças vêm do ruído óptico configurado.
   Erros baixos nesta simulação não são uma medição da precisão física.

`setCameraConnected(frontCamera, connected)` permite controlar a entrega de uma
câmera em cenários/testes Java. O teste desliga o IO; os diagnósticos ópticos do
simulador podem continuar mostrando o que ela enxergaria. Não é um botão no
AdvantageScope nem reproduz uma falha física de rede/energia.

## Validação automatizada

Os testes de Drive verificam inicialização única, movimento por motores virtuais,
Disabled e independência da referência após correção/seed de visão.
Os testes PhotonLib/OpenCV verificam a tag 25 passando da câmera frontal para a
traseira após uma rotação, perda de alvos e poses calculadas a partir da geometria.
Um teste completo introduz erro em X/Y, verifica sua correção com a frontal sozinha,
desliga as duas câmeras, introduz deriva de encoder e verifica a recuperação pela
traseira sozinha, sem reset automático do Drive.

Os testes de entrada/fusão verificam todos os frames de um lote, ordem temporal,
deduplicação, ambiguidade/altura/inclinação/limites do campo, entrega de uma câmera
durante falha da outra, preservação do heading e compatibilidade Limelight.
O AdvantageKit é exercitado com gravação e recuperação dos records de observação
em `LogTable`, incluindo lote vazio e entrega ao Drive. O teste isolado
`visionReplayTest` também grava e lê um arquivo WPILOG, executa a fusão com o
relógio do arquivo e confere saídas originais e recalculadas no resultado.
O histórico testa interpolação, expiração e relógio reiniciado.

## Referências

- [Simulação PhotonLib 2026.3.4](https://docs.photonvision.org/en/v2026.3.4/docs/simulation/simulation-java.html)
- [Especificações da Limelight 4](https://docs.limelightvision.io/docs/docs-limelight/getting-started/limelight-4)
- [Vision Template do AdvantageKit](https://docs.advantagekit.org/getting-started/template-projects/vision-template/)
- [PhotonPoseEstimator 2026.3.4](https://docs.photonvision.org/en/v2026.3.4/docs/programming/photonlib/robot-pose-estimator.html)
- [SwerveDriveOdometry da WPILib](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/kinematics/SwerveDriveOdometry.html)
