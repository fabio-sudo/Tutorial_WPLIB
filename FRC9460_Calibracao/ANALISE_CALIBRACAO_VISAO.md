# Análise do projeto de calibração — 07/10/2026

> Histórico da análise inicial de 07/10/2026. Depois foram implementadas as correções de freio/yaw e a validação temporal da visão. Consulte [GUIA_FREIO_E_YAW.md](GUIA_FREIO_E_YAW.md) e a atualização de acompanhamento ao final deste arquivo para distinguir propostas antigas, resultados observados e testes pendentes.

Objetivo confirmado: usar o AdvantageScope para medir e melhorar a precisão da visão e do swerve, aceitando medições confiáveis sem impedir a recuperação da posição. Este documento registra a análise e a proposta; não altera o código do robô.

## Conclusão sobre os bloqueios

É plausível que um corte rígido por diferença entre visão e odometria tenha impedido a aquisição inicial. Exemplo: odometria inicia em (0, 0), robô está em (5, 3), câmera informa a posição correta; um corte de 1 m rejeita justamente a correção necessária. Não há logs antigos disponíveis neste projeto para confirmar que essa foi a causa observada.

Na versão atual, `Vision.java` já não rejeita por distância nem por `PoseErrorMeters`. `OutlierSuspected` é apenas diagnóstico. Isso permite recuperar a posição, mas também deixa passar saltos com números válidos. Reduzir o peso de uma medição ruim diminui seu efeito; não garante eliminá-lo.

| Condição atual | Efeito |
| --- | --- |
| IDs 25, 26, 9 e 10 | Lista enviada à câmera; outras tags ficam fora da seleção de localização |
| Nenhuma tag na estimativa | Rejeita |
| Pose inexistente ou números inválidos | Rejeita |
| Distância média não finita | Rejeita; distância negativa ainda não é rejeitada |
| Timestamp não finito ou não positivo | Rejeita |
| Velocidade angular não finita ou módulo >= 720 graus/s | Rejeita |
| Timestamp não avançou | Não adiciona a mesma medição novamente |
| Distância >= 4 m | Apenas diagnóstico |
| Divergência > 1 m e intervalo entre frames <= 0,25 s | Apenas diagnóstico de possível outlier |
| Uma tag | Pode participar da fusão automática |
| Seed manual | Exige pelo menos duas tags, frame recebido recentemente e robô parado |

`SeedReady = false` não significa necessariamente que a fusão esteja bloqueada. `MeasurementAccepted` e `NewMeasurement` são eventos de um ciclo: podem alternar entre verdadeiro e falso quando a câmera atualiza mais devagar que o robô.

## Problemas confirmados e prioridades

### 1. Direção do Pigeon e referência do campo — corrigir primeiro

`Drive.zeroHeading()` chama `gyroIO.resetYaw()`, mas `GyroIOPigeon2` não implementa esse método. A interface fornece uma implementação vazia. O código zera os valores locais, porém a próxima leitura volta a trazer o yaw anterior do sensor. A simulação implementa o reset, portanto pode esconder esse defeito.

O MegaTag2 recebe diretamente `drive.getRotation()`, que representa o yaw do Pigeon. É necessário garantir que esse ângulo corresponda à orientação no campo. O seed atual só muda X/Y e mantém o yaw; ele não resolve uma referência angular errada.

A documentação do MegaTag2 define zero como o robô apontado para a parede da aliança vermelha, no referencial azul, com rotação anti-horária positiva. A montagem traseira de 180 graus da câmera é uma configuração separada. Zerar o robô em qualquer direção não estabelece automaticamente o zero do campo. [Referência Limelight](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2).

Proposta: corrigir o reset real, verificar seu resultado e separar direção de campo de eventual referência de condução do piloto. Não ajustar filtros para compensar yaw incorreto.

### 2. Saúde e idade dos dados — indicadores atuais podem enganar

- `GyroIOPigeon2` grava `connected = true` incondicionalmente. Não detecta perda de comunicação, e a visão não consulta esse estado.
- A validação do timestamp verifica apenas se ele é finito e positivo. Falta rejeição explícita de medição antiga ou futura, com tolerâncias compatíveis com a latência medida.
- `FrameAgeSeconds` e `RawFrameAgeSeconds` usam o instante local de processamento. Não medem a idade desde a captura. O helper já fornece timestamp descontando a latência.
- Um frame congelado não entra repetidamente, pois há deduplicação; mesmo assim, `Valid` pode continuar verdadeiro com dados antigos.
- `MeasurementAccepted` é marcado antes da chamada ao estimador. Uma medição antiga pode ser ignorada internamente pela WPILib. Portanto esse indicador não comprova que houve correção efetiva.
- `VisionGapSeconds` só avança nos frames que passam pelos filtros básicos: mede continuidade de frames válidos, não necessariamente conexão da câmera.

Proposta: registrar idade de captura, idade da última publicação/heartbeat, saúde real do gyro, motivo de rejeição e contadores por motivo. Separar frame recebido, aprovado pelos filtros e enviado ao estimador. Na WPILib 2026.2.1 o histórico usado pelo estimador dura 1,5 s e medições anteriores podem ser ignoradas. [Código da versão usada](https://raw.githubusercontent.com/wpilibsuite/allwpilib/v2026.2.1/wpimath/src/main/java/edu/wpi/first/math/estimator/PoseEstimator.java).

### 3. Comparação independente para calibrar

`Robot/Pose` é a pose do `SwerveDrivePoseEstimator`, já corrigida pela câmera. Não existe uma pose paralela só de rodas + gyro. O teste de 2 m usa a pose fundida, então uma correção visual pode mudar a distância que o teste pensa ter percorrido.

Proposta: manter e registrar três resultados simultâneos:

1. Odometria apenas de rodas + gyro, implementada como uma odometria independente.
2. Pose recebida da Limelight, antes dos filtros, quando numericamente válida.
3. Pose fundida usada pelo robô.

Comparar com uma quarta referência: posição física medida do centro do robô. Duas estimativas concordarem entre si não prova que estejam certas. Corrigir raio da roda usando a câmera como única referência pode transferir o erro da câmera para a odometria.

`PoseErrorMeters` compara a câmera do passado com a pose atual. Em movimento, parte da diferença pode ser apenas latência. Se esse valor voltar a participar de um filtro, comparar com o estado estimado no instante da captura.

### 4. Configuração da câmera e do ambiente

- A lista `{25, 26, 9, 10}` continua sendo uma restrição real. Confirmar que as tags do teste estão nela e no mapa carregado.
- Tags deslocadas em relação ao mapa produzem erro sistemático, mesmo com imagem boa. Em uma sala de testes, o mapa precisa representar as posições reais das tags usadas.
- Conferir fisicamente os valores configurados: traseira de 0,29 m, direita de 0,16 m, altura de 0,46 m, pitch de 4 graus e yaw de 180 graus. O código envia esses valores à câmera.
- `IMU_MODE = 0` está declarado, mas não é aplicado com `SetIMUMode`. Confirmar modelo da câmera e configuração efetiva antes de adotar uma política de IMU.
- O helper local declara exigir LLOS 2026.0 ou posterior; verificar o firmware instalado.

### 5. Outros pontos do projeto que afetam a calibração

| Ponto | Consequência / melhoria |
| --- | --- |
| Raio da roda duplicado em `Constants` e `ModuleIOTalonFX` | Hoje ambos são 0,0508 m; alterar só um gera inconsistência entre comando e distância medida. Usar uma fonte única |
| `GyroScalarZ = -5.48` | Comentários registram outra medição aproximada. Revalidar por voltas físicas medidas nos dois sentidos; não escolher sinal apenas pelo comentário. A unidade é graus por volta, conforme a API CTRE |
| `Rotate360Command` solicita 10 voltas | Nome pode induzir teste errado. Usa valor absoluto da rotação, podendo mascarar sentido invertido. Explicitar quantidade/sentido e reduzir a primeira validação para uma volta |
| `A` e `Back + A` têm comandos separados | Podem disputar o Drive dependendo da ordem de acionamento. Tornar combinações mutuamente exclusivas |
| Controles sem condição de modo | Joystick e botões podem atuar em autônomo quando o Drive está disponível. Restringir comandos de operador ao modo pretendido |
| Autônomo ainda é `Autos.exampleAuto` | Não executa trajetória do swerve. É um item para evolução futura; não explica rejeição de poses |
| `driveSpeedTest` ativa steer direto e logo o desativa ao pedir velocidade | Não implementa a espera de alinhamento sugerida pelo comentário. Separar preparação e medição se esse teste for usado |
| Configurações CTRE sem checagem dos retornos | Falha ao aplicar configuração pode passar despercebida; registrar status e alertas úteis |
| Simulação com ganhos diferentes do hardware | Útil para lógica, mas não comprova ganho/calibração física do robô |
| Visão lida diretamente por NetworkTables | Não há `VisionIO` nem configuração de replay; `replayWatch` sozinho não habilita replay determinístico |
| Arquivos muito extensos por espaçamento e comentários repetidos | Simplificar gradualmente após estabilizar comportamento, preservando comentários sobre unidades e decisões |

[Unidades do ajuste do Pigeon — CTRE](https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/configs/GyroTrimConfigs.html).

## Estratégia proposta para filtrar sem travar

Separar aquisição inicial/recuperação de acompanhamento normal:

- Na aquisição, permitir grande diferença de X/Y se houver orientação de campo confiável e uma sequência de frames consistentes com o robô parado. Não exigir proximidade da pose inicial (0, 0). A quantidade de tags influencia confiança; exigir duas para todo frame impediria operação válida com uma tag.
- No acompanhamento, avaliar inovação no timestamp correto, movimento medido e qualidade observada. Um salto isolado pode ser rejeitado; um conjunto estável divergente deve abrir uma decisão de recuperação, não um bloqueio eterno.
- Para grande reposicionamento, exigir robô parado ou uma etapa explícita de recuperação; não fazer reset brusco no meio de movimento controlado.
- Manter filtros para ausência de informação, dados inválidos, dados antigos e gyro inadequado. Limites de distância/rotação e pesos devem vir dos testes, não de números escolhidos para deixar o indicador verde.
- Ajustar `XYStdDev` com os resultados por distância e quantidade de tags. A tabela atual satura acima de 3 m: uma observação a 4 m e outra a 10 m recebem o mesmo peso se tiverem a mesma categoria de tags.

Esta é uma proposta de implementação. Não há novo filtro aplicado nesta análise.

## AdvantageScope: primeira coleta usando o código atual

Pode começar com o robô desabilitado, parado e vendo uma tag permitida, sem acionar seed ou zero heading durante a coleta. A posição da tag precisa corresponder ao mapa.

No painel de campo 2D, sobrepor `Robot/Pose` e `Vision/Rear/MegaTag2/Pose`. Usar o mesmo campo, unidades e origem. No log do AdvantageKit, saídas podem aparecer sob `RealOutputs`; localizar pelo final do nome. O painel suporta múltiplas poses. [Documentação do campo 2D](https://docs.advantagescope.org/tab-reference/2d-field/).

Registrar 30 segundos e observar:

| Sinal | O que verificar |
| --- | --- |
| `Vision/Rear/HasTarget` e `TagID` | Alvo detectado e ID esperado; não são prova de pose aprovada |
| `Vision/Rear/RobotYawDegrees` | Orientação compatível com a direção física no campo |
| `Vision/Rear/MegaTag2/TagCount` e `AvgTagDist` | Quantidade e distância coerentes |
| `Vision/Rear/MegaTag2/TimestampSeconds` | Avança ao longo do tempo |
| `Vision/Rear/MegaTag2/NewMeasurement` e `MeasurementAccepted` | Pulsos continuam chegando |
| `Vision/Rear/MegaTag2/Reject/*` | Qual filtro reprova quando há falha |
| `Vision/Rear/MegaTag2/FrameAgeSeconds` | Cresce se não chegam medições aceitas; lembrar que não é idade de captura |
| `Vision/Rear/MegaTag2/XYStdDevMeters` | Peso escolhido pelo código, não precisão medida |
| `Vision/Rear/MegaTag2/PoseErrorMeters` | Divergência entre estimativas, não erro absoluto de localização |

Anotar junto ao log: modelo e firmware da Limelight, mapa utilizado, IDs visíveis, posição/orientação física do centro do robô e distância aproximada às tags. Não concluir precisão apenas pela estabilidade do desenho no campo.

## Sequência de validação após instrumentação

1. **Parado:** repetir 30 s a aproximadamente 1, 2, 3 e 4 m, com uma tag e depois múltiplas tags onde possível. Medir erro médio em X/Y, erro de posição no percentil 95, dispersão e taxa de rejeição por motivo. Essas distâncias são pontos de coleta, não novos cortes.
2. **Referência angular:** verificar direções conhecidas de 0, 90, 180 e -90 graus. Confirmar sinal e orientação de campo antes de ajustar pesos.
3. **Perda e retorno:** cobrir o campo de visão por alguns segundos e descobri-lo. Verificar estado sem alvo, envelhecimento dos dados e retomada. Testar desconexão com o robô desabilitado para diferenciar câmera congelada de ausência de tags.
4. **Translação física:** após corrigir conflitos dos controles, fazer percurso medido de 2 m em baixa velocidade e retorno. Usar odometria independente para calibração da roda e analisar visão separadamente.
5. **Rotação:** começar com uma volta física marcada em cada sentido. Verificar erro final e deslocamento indevido de X/Y. O teste atual chamado `Rotate360Command` faz dez voltas e não deve ser confundido com este procedimento.
6. **Recuperação:** iniciar com X/Y estimado propositalmente divergente, mantendo heading correto, e verificar a aquisição sem travamento. Depois testar salto isolado de visão por injeção em teste automatizado, sem precisar enganar fisicamente a câmera.

Definir tolerâncias finais depois da coleta e conforme a necessidade do robô. Precisão absoluta, dispersão e capacidade de recuperar após perder tags são critérios distintos.

## Ordem recomendada de implementação

1. Corrigir reset/saúde do gyro e garantir referência angular; registrar falhas de configuração.
2. Adicionar odometria independente e diagnóstico de timestamps, estado da câmera e rejeições. Preservar as chaves atuais usadas no AdvantageScope.
3. Isolar decisão de aceitação em lógica testável e criar entrada de visão gravável para replay. Testar frames antigos/futuros/repetidos, perda de gyro, ausência de tags, aquisição longe da origem e recuperação após perda.
4. Executar a primeira coleta e ajustar política/pesos a partir dos dados. Uma alteração por rodada, repetindo a mesma condição física.
5. Unificar constantes e corrigir comandos de calibração. Evoluir autônomo em uma etapa posterior.

## Verificação realizada

Inspeção de fontes de visão, integração LimelightHelpers, Drive, módulos reais/simulados, gyro real/simulado, comandos, calibração, inicialização/logging e configuração do build. `gradlew test --offline --no-daemon` concluiu com `BUILD SUCCESSFUL`, usando os artefatos de compilação já atualizados. As tarefas de testes retornaram `NO-SOURCE`: não existe suíte automatizada. Não foi executado teste físico, deploy nem alteração de ganhos/filtros.

## Atualização de acompanhamento — 08/10/2026

Os resultados físicos abaixo foram relatados pelo usuário ou observados nos prints e na gravação compartilhados. Não houve análise completa de um arquivo WPILOG desses ensaios. A calibração da odometria já havia sido concluída pelo usuário; o trabalho atual concentra-se na visão e na fusão com o PoseEstimator.

| Etapa | Situação e evidência |
| --- | --- |
| Freio em X e referências de yaw | Implementados; a compilação e os 11 testes existentes de controles do Drive passaram na verificação posterior à análise inicial. |
| Correção automática com visão | O usuário confirmou que a pose visual e `Robot/Pose` retornaram à referência física após o percurso. |
| Ausência de tags | Com a câmera coberta, `TagCount = 0`, `Valid = false` e nenhuma nova medição foi enviada à fusão. |
| Confiança dinâmica | Na gravação, a passagem de duas tags para uma coincidiu com a mudança de StdDev de 0.40 para 0.85. Isso confirma a seleção do peso nesse caso, não a precisão absoluta do robô. |
| Estabilidade com o robô parado | Nas nove amostras consultadas na gravação, a amplitude de X foi aproximadamente 1,5 cm e a de Y, 3,1 cm. Não são os extremos de todos os frames. |
| Aceitação temporal normal | Com limite de 0.250 s, as capturas observadas ficaram abaixo do limite, com os filtros temporais em false e medições novas sendo processadas. |
| Rejeição por idade | Com limite temporário de 0.005 s e duas tags detectadas, uma captura de aproximadamente 42 ms foi rejeitada: `StaleMeasurement = true` e a fusão não recebeu a medição. |
| Retomada após o ensaio | O limite foi restaurado no código para 0.250 s; no print posterior, `Valid` voltou a true, os filtros temporais ficaram em false e os pulsos de novas medições retornaram. |

### Testes pendentes

- [ ] Verificar X/Y durante pequenos giros no próprio eixo, mantendo o centro físico na mesma marca e as tags visíveis. **Adiado a pedido do usuário por falta de condições para executar o teste agora.** Esse ensaio continua necessário para conferir a configuração da câmera durante a rotação.
- [ ] Comparar X/Y com uma medição física independente do centro do robô nas referências do campo. Concordância entre as duas poses não comprova precisão absoluta.
- [ ] Exercitar a rejeição de timestamp futuro acima da tolerância de 0.020 s, usando entrada controlada em software. Os prints normais confirmam apenas que esse filtro ficou inativo com os timestamps observados.
- [ ] Ampliar a coleta por distância, quantidade de tags e movimento antes de considerar os pesos definitivos para competição.

### Evolução da arquitetura da visão

O usuário adicionou os logs de limites de idade de captura, tolerância futura e velocidade angular dentro do `periodic()` de `Vision.java`. Eles permitem conferir os valores usados pelo código executado no robô, inclusive durante ensaios com limites temporários. O `CaptureAgeSeconds` mede a idade da captura; `RawFrameAgeSeconds` mede o tempo local desde o último processamento de uma pose visual aprovada pelos filtros.

O próximo trabalho, solicitado pelo usuário, é organizar a visão em `VisionIO`, implementações para Limelight traseira e frontal, simulação e replay pelo AdvantageKit. O usuário continuará aplicando os blocos de código, com explicação e revisão de cada etapa. Os testes físicos permanecem pendentes por falta de condições no ambiente atual.

Sequência prevista:

1. Criar o contrato `VisionIO` com dados de entrada `@AutoLog`, incluindo os timestamps de captura e leitura.
2. Mover acesso aos helpers para uma implementação comum `VisionIOLimelight`, reutilizada por `VisionIOLimelightRear` e `VisionIOLimelightFront`. A configuração física de cada câmera será fornecida separadamente.
3. Preparar `VisionIOSim` com dados de simulação independentes da pose fundida que está sendo avaliada.
4. Fazer `Vision` consumir exclusivamente entradas processadas por `Logger.processInputs`, mantendo filtros, confiança e fusão no núcleo e preservando os logs usados no AdvantageScope.
5. Configurar seleção REAL/SIM/REPLAY no ponto de composição e a fonte `WPILOGReader` antes de `Logger.start()`. Durante replay, as entradas serão preenchidas a partir do log, com IO sem acesso a câmeras reais.

O usuário criou `VisionIO.java`, `VisionIOLimelight.java` e `VisionIOLimelightRear.java`; a compilação passou após cada etapa e o processador de anotações gerou `VisionIOInputsAutoLogged`. A implementação comum lê dados pelos helpers e diagnostica atividade pelo heartbeat, separada dos filtros de aceitação da pose. A classe traseira reutiliza os valores atuais de `Constants.LimelightConstants`. A classe frontal, a simulação e a integração do contrato com `Vision` ainda estão pendentes.

O próximo bloco orientado será `VisionIOLimelightFront`, recebendo nome, posição física, orientação e IDs pelo construtor. O projeto ainda não contém configuração física de uma câmera frontal; esses valores deverão ser fornecidos antes de instanciá-la para uso real. A preparação da classe não confirma instalação nem calibração dessa câmera.

Os logs antigos continuam úteis para análise no AdvantageScope; o replay da nova camada de visão precisará das entradas gravadas após a integração do IO. A comparação temporal deverá usar dados de tempo registrados ou relógio determinístico do AdvantageKit, para evitar que o tempo real do computador altere as decisões no replay.
