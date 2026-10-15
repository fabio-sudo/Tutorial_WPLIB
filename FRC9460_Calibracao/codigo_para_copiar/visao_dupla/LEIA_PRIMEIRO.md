# Vision e Drive para duas Limelights

Estes arquivos foram preparados como propostas completas para você copiar. Na entrega inicial, os arquivos em `src` não foram substituídos. Os marcadores de alteração `NEW` e os ícones correspondentes foram removidos dos comentários.

Atualização de 09/10/2026: após a cópia feita pelo usuário e com sua autorização, foram corrigidos diretamente em `src` a recuperação após timestamp futuro rejeitado e o envio de `IMU_MODE` no construtor de `VisionIOLimelight`. Os testes de visão também foram incluídos em `src/test`. Esta cópia de `Vision.java` já contém a correção de timestamp.

## Como colocar no projeto

1. Copie todo o conteúdo de `Vision.java` desta pasta para `src/main/java/frc/robot/subsystems/vision/Vision.java`, substituindo o conteúdo antigo.
2. Copie todo o conteúdo de `Drive.java` desta pasta para `src/main/java/frc/robot/subsystems/drive/Drive.java`, substituindo o conteúdo antigo.
3. Mantenha `VisionCamera`, `VisionIO`, `VisionIOLimelight`, `VisionIOLimelightRear`, `VisionIOLimelightFront` e `VisionIOSim` que já estão no projeto. `VisionIOInputsAutoLogged` continua sendo gerado pelo AdvantageKit.
4. O `RobotContainer` pode continuar usando `new Drive()` e `new Vision(drive)`.
5. Compile antes de enviar ao robô. Não copie os dois arquivos novamente para outra pasta dentro de `src/main/java`, pois isso criaria classes duplicadas.

## Ativar a frontal

No começo do Vision há:

```java
private static final boolean FRONT_CAMERA_ENABLED = false;
```

A traseira funciona com esse valor em `false`. A frontal está implementada, mas seus `FRONT_*` ainda representam uma montagem provisória. Depois de medir posição e ângulos reais, corrigir as constantes e conferir mapa e nome de rede, altere para `true` e recompile. Com `true`, se qualquer câmera ficar desconectada ou sem tags, a outra continua funcionando sozinha.

Para montar explicitamente somente a frontal, também é possível usar no RobotContainer:

```java
new Vision(drive, new VisionCamera("Front", new VisionIOLimelightFront()));
```

Esse exemplo exige importar as duas classes adicionais no RobotContainer. Os getters antigos sem argumento consultam a traseira; em uma configuração exclusivamente frontal, use os getters com argumento, como `hasTarget("Front")`. O comando de seed seleciona automaticamente a câmera elegível e não depende do nome Rear.

## Comportamento das câmeras

| Situação | Comportamento |
| --- | --- |
| Rear vê tag 9 e Front vê tag 25 ao mesmo tempo | As duas poses válidas podem entrar no mesmo estimador. |
| Cada câmera vê duas ou mais tags diferentes | Mesmo comportamento; a confiança é calculada separadamente. |
| As duas enxergam a mesma tag | Podem contribuir, com redução conservadora do peso quando estão no mesmo grupo temporal. |
| Uma câmera não foi configurada, está desligada ou não vê tags | A outra funciona sozinha; não há exigência de duas câmeras para aceitar visão. |
| As duas ficam sem leitura | O Drive continua atualizando a odometria. |
| A mesma câmera repete um frame | O timestamp impede aplicá-lo novamente. |
| Uma captura chega antes de outra mais antiga | A fila tenta ordenar por captura. Se a antiga chegar depois de uma correção já aplicada, ela é descartada com `OUT_OF_ORDER`. |
| Poses próximas no tempo divergem muito | O grupo conflitante é rejeitado com `CAMERAS_DISAGREE`. A próxima leitura coerente pode voltar a entrar. |

Somente o Drive contém o estimador. As câmeras não sobrescrevem a pose uma da outra e não fazem reset automático. O yaw utilizado na fusão continua sendo o heading do estimador ligado ao Pigeon; a orientação calculada pelo MegaTag2 não é usada como segunda referência angular independente.

O filtro de IDs permanece o que você configurou nos IOs: atualmente `{25, 26, 9, 10}`. Tags diferentes podem colaborar desde que sejam permitidas e pertençam ao mapa usado pelas câmeras. As duas devem usar o mesmo referencial azul e a montagem real de cada câmera. Essas condições vêm da [documentação do MegaTag2](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2).

## Decisões que precisam ficar documentadas

- **Fila:** `REORDER_SECONDS = 0.080` e `PAIR_WINDOW_SECONDS = 0.020`. O primeiro frame de um grupo normalmente é processado quando tem pelo menos cerca de 100 ms desde a captura, mais o arredondamento de um ciclo. Não é uma espera por conexão da outra câmera. A odometria continua a cada ciclo. Isso troca um pouco de rapidez da correção por melhor ordenação; deve ser avaliado nos testes em movimento.
- **Ordenação:** a WPILib 2026.2.1 remove atualizações visuais posteriores ao inserir uma medição mais antiga. A proteção contra atraso excessivo evita perder uma correção já aplicada. A referência é a [implementação da versão usada no projeto](https://github.com/wpilibsuite/allwpilib/blob/v2026.2.1/wpimath/src/main/java/edu/wpi/first/math/estimator/PoseEstimator.java).
- **Divergência:** o limite inicial é `0.15 + 3.0 * hypot(desvioA, desvioB)`, em metros. Para duas medições com desvio base de 0,25 m, fica aproximadamente 1,21 m. Compara-se o mesmo instante usando o movimento estimado entre capturas. Isso detecta divergências grandes; não prova qual câmera está certa e não garante rejeitar toda pose ruim. Se uma montagem estiver errada, corrija-a em vez de aumentar esse limite para esconder o problema.
- **Confiança:** foram mantidas as faixas de distância e quantidade de tags do projeto. Num grupo com duas câmeras, cada desvio é multiplicado por `sqrt(2)`. Com uma só câmera contribuindo no grupo, não há esse multiplicador. É uma escolha conservadora inicial, ainda sem calibração estatística em campo.
- **Tags compartilhadas:** o VisionIO atual traz somente o ID principal, não todos os IDs utilizados na pose. Portanto o código não identifica precisamente a sobreposição de tags. A redução conservadora do peso vale para grupos com duas câmeras, inclusive quando seus IDs principais diferem.
- **Distância e erro relativo:** não foi reintroduzido um bloqueio automático por ultrapassar 4 m, nem por a visão estar longe da odometria. Esses valores continuam como diagnóstico. `PoseErrorMeters` é diferença em relação à estimativa histórica, não erro medido contra uma posição física conhecida.
- **Conexão:** `Camera/Connected` é diagnóstico do heartbeat. Não depende de enxergar tags. A admissão de medições depende dos dados, timestamps, duplicação e gyro, evitando depender exclusivamente desse indicador.
- **Seed manual:** exige robô parado e uma medição recente, coerente, já aplicada e com pelo menos duas tags. É a ação explícita já ligada ao controle; não acontece no periodic. Preserva o heading e invalida a fila anterior ao reset.
- **Limites existentes:** idade de captura, tolerância de timestamp futuro e velocidade angular continuam vindo de `LimelightConstants`. O limite de rotação atual do projeto não deve ser apresentado como recomendação oficial da Limelight.

## Logs principais

As entradas ficam em `Vision/Rear/Inputs` e `Vision/Front/Inputs` quando a respectiva câmera está configurada. No AdvantageScope de execução real, normalmente aparecem abaixo de `RealOutputs` os registros produzidos por `recordOutput`.

Dentro de `Vision/<câmera>/MegaTag2`, acompanhe:

- `Valid`: a leitura atual passou pelos filtros básicos.
- `NewMeasurement`: entrou um frame novo e válido neste ciclo.
- `MeasurementAccepted`: pelo menos uma medição da câmera foi aplicada neste ciclo. Por causa da fila, não é necessariamente o mesmo ciclo de `NewMeasurement`.
- `FusionResult`: resultado da última operação do ciclo, como `QUEUED`, `ACCEPTED`, `REPEATED_FRAME`, `OUT_OF_ORDER`, `CAMERAS_DISAGREE` ou `NO_ODOMETRY_AT_CAPTURE`.
- `AcceptedCount`, `QueuedCount`, `RejectedCount`: contadores que facilitam confirmar a continuidade sem depender de pulsos.
- `BaseXYStdDevMeters`: confiança calculada a partir da leitura.
- `XYStdDevMeters`: confiança efetivamente utilizada na última aplicação.
- `Reject/*`: causas de rejeição de entrada e divergência entre câmeras.

`Drive/Vision/XYStdDevMeters` é mantido por compatibilidade, mas representa somente a última medição aplicada. Para analisar duas câmeras, use os caminhos individuais.

As poses visuais podem permanecer no último valor quando a câmera perde tags. Consulte `Valid` e os contadores junto. `RawFrameAgeSeconds` continua sendo idade desde processamento; `SeedCaptureAgeSeconds` indica a idade desde a captura usada pelo seed.

## Simulação, replay e verificação

A compilação e **32 testes de software passaram no projeto após a revisão de 09/10/2026**: os 11 testes existentes de freio/yaw e 21 cenários de visão. Foram exercitados timestamps iguais, tags iguais/diferentes, câmeras isoladas, recuperação de leitura, frames repetidos, ordem invertida entre ciclos, atraso excessivo, divergência e recuperação, timestamps inválidos por idade/futuro, NaN, reset, yaw e seed manual. Os dois cenários adicionais verificam recuperação da mesma câmera após um timestamp futuro rejeitado e a rejeição continuada desse frame quando ele permanece congelado.

`VisionMultiCameraTest.java` contém os cenários de visão e já foi incluído em `src/test/java/frc/robot/subsystems/vision/`. Também há uma cópia atualizada nesta pasta. As verificações isoladas anteriores ficaram em `build/verificacao_visao_dupla` e `build/revisao_20261009`; nenhum deploy foi executado.

`VisionIOSim` aceita frames controlados, mas não gera imagens nem detecções geométricas automaticamente. Os construtores permitem injetar esse IO para testes.

Os dois subsistemas escolhem IOs vazios quando `Logger.hasReplaySource()` é verdadeiro. Isso prepara a separação de entradas; **não ativa replay sozinho**. O Robot.java atual ainda precisa configurar a fonte de replay. Para reproduzir visão serão necessários logs que contenham os novos caminhos `Vision/<câmera>/Inputs`, com a mesma configuração de câmeras. Logs antigos apenas com saídas não bastam. O relógio da lógica segue as [orientações de timestamps determinísticos do AdvantageKit](https://docs.advantagekit.org/theory/deterministic-timestamps/).

A verificação em software não substitui os testes físicos: montagem da frontal, precisão contra referências medidas, perdas e retomadas durante movimento, rotação e efeito da fila continuam pendentes para um ambiente adequado.
