# Virtualização do Swerve e das câmeras

## Etapa implementada

O robô virtual começa em **X = 2 m, Y = 2 m, orientação = 0°**.
O Drive atualiza uma segunda `SwerveDriveOdometry` usando os encoders e o
giroscópio simulados. Ela movimenta as câmeras PhotonLib e não recebe correções
da visão. O estimador normal do Drive continua separado.

Essa referência é cinemática: compartilha os sensores virtuais com o estimador.
Não modela derrapagem, contato com obstáculos ou colisões. Portanto, erro quase
zero entre as duas odometrias não comprova a precisão do robô físico.

**Fusão PhotonLib desligada nesta etapa.** As poses PhotonLib são diagnósticos.
As antigas medições artificiais fixas foram removidas. Não espere crescimento
dos contadores de aceitação em `Vision/Rear/MegaTag2` ou `Vision/Front/MegaTag2`.
O sinal `Vision/Simulation/FusionEnabled` deve estar `false`.

O robô físico mantém seus IOs Limelight. O cenário não é criado durante replay.
Este trabalho não configura uma fonte de replay nem gravação desktop de WPILOG.

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

Cada câmera tem os diagnósticos abaixo em
`Vision/Simulation/Photon/Front/` e `Vision/Simulation/Photon/Rear/`:

| Campo | Significado |
| --- | --- |
| `TagCount`, `TagIds` | Tags detectadas no último frame |
| `TagPoses` | Poses das tags detectadas; use objeto Vision Target no Campo 3D |
| `HasNewFrame`, `ProcessedFrameCount` | Chegada e processamento dos frames |
| `HasEstimatedPose`, `UsedTagCount` | Resultado do cálculo de pose |
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
- Resolução 960×720, FOV diagonal 90°, 25 FPS, latência 30 ms e ruído em pixels
  são parâmetros de ensaio, não uma calibração das câmeras físicas.
- Não há simulação de iluminação real, nem garantia de oclusão pelas estruturas
  desenhadas pelo AdvantageScope. O modelo 3D do campo é uma visualização.
- PhotonPoseEstimator multitag com fallback de menor ambiguidade não é MegaTag2.
  Antes de habilitar fusão, será necessário tratar todos os frames, timestamps,
  filtros de IDs e confiança de forma explícita.

## Validação automatizada

Os testes de Drive verificam inicialização única, movimento por motores virtuais,
Disabled e independência da referência após correção/seed de visão.
O teste PhotonLib/OpenCV verifica a tag 25 passando da câmera frontal para a
traseira após uma rotação, perda de alvos fora de alcance e ausência de medições
artificiais nos IOs. O histórico testa interpolação, expiração e relógio reiniciado.
Os testes existentes de controle e visão dupla também devem continuar passando.

## Referências

- [Simulação PhotonLib 2026.3.4](https://docs.photonvision.org/en/v2026.3.4/docs/simulation/simulation-java.html)
- [Vision Template do AdvantageKit](https://docs.advantagekit.org/getting-started/template-projects/vision-template/)
- [SwerveDriveOdometry da WPILib](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/kinematics/SwerveDriveOdometry.html)
