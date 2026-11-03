# Freio em X e referências de yaw

Implementado em 07/10/2026. Esta etapa prepara os controles e o gyro para calibração. Os filtros de `Vision.java` e o `LimelightHelpers.java` não foram editados nesta etapa.

## Controles

| Ação | Comando | Condições |
| --- | --- | --- |
| Manter rodas em X | Segurar seta para baixo, sem Start | Teleoperado habilitado |
| Liberar controle ao piloto | Soltar seta para baixo | Uma calibração interrompida não reinicia sozinha |
| Zerar referência do piloto e contador de voltas | Y, sem Start e sem Back | Parado, gyro saudável; desabilitado ou teleoperado |
| Definir zero de orientação do campo | Start + Y | Desabilitado, parado e gyro saudável; alinhar fisicamente a frente do robô ao +X do campo azul antes de usar |
| Definir 180° de orientação do campo | Start + A | Mesmas condições; a frente do robô aponta para a parede da aliança **azul** (−X). É a posição do treino com a Limelight traseira vendo o hub azul (tags 25/26) |
| Seed manual de X/Y pela visão | Start + seta para baixo | Mantém o acionamento e os requisitos já existentes no projeto |

O freio em X solicita velocidade zero e posiciona os módulos nas diagonais. Outros comandos que exigem o Drive não podem interrompê-lo enquanto ele estiver agendado. O comando é cancelado ao soltar o botão, sair do teleoperado ou desabilitar. Ao pressionar Start, a combinação passa a ser a do seed e deixa de ser o freio. Ele não substitui o Disable da Driver Station nem é um travamento mecânico com o robô desligado.

Os limitadores do joystick são zerados ao sair do freio. Com analógicos centralizados, não há retomada de um comando antigo. Se o piloto mantiver um analógico deslocado, a condução será retomada ao liberar o freio. As rodas podem permanecer em X até uma nova ordem de movimento.

## O que foi corrigido no yaw

Antes, `Drive.zeroHeading()` chamava um `resetYaw()` vazio no Pigeon real e escrevia zero nas entradas locais. A próxima leitura trazia o ângulo anterior, tornando o reset inconsistente. A simulação tinha um reset próprio, diferente do comportamento real.

Agora a leitura bruta do sensor é preservada. A orientação de campo é redefinida com `poseEstimator.resetPosition()`, passando o ângulo bruto real, as posições dos módulos e a nova pose. A WPILib mantém o offset; o zero continua correto nos ciclos seguintes. Esse é o princípio usado no template AdvantageKit.

Exemplo: o Pigeon lê 73 graus. Com o robô fisicamente alinhado ao +X, Start + Y define o campo como zero. O sensor pode continuar mostrando 73 graus, mas `FieldYawDegrees` mostra zero. Depois de girar fisicamente +90 graus, o heading de campo passa a +90 graus.

**Por que isso importa para a visão (teste de 28/10/2026).** O MegaTag2 usa o
yaw enviado pelo robô como verdade. Ao ligar, o Pigeon vale 0°. Se o robô estiver
de costas para o hub azul (frente para a parede azul, 180°), a Limelight calcula a
posição espelhada, atrás das tags, e o robô aparece no meio da arena. Antes de
habilitar, com o robô parado: frente para +X → Start + Y; frente para a parede
azul → Start + A. Tags 25 e 26 ficam na face do hub azul virada para a parede
azul (x = 4,02 m), então o robô certo aparece com x menor que 4 m.

Y sozinho muda somente a referência do piloto. Exemplo: o robô está a 90 graus no campo; apertar Y deixa `DriverYawDegrees` em zero, mas `FieldYawDegrees` permanece em 90 graus. A visão continua recebendo a orientação de campo.

**Frente do piloto, sem a Driver Station.** A direção não usa a aliança da Driver Station. Com o robô desabilitado, até o primeiro enable, o lado vem da pose corrigida pelas tags: na metade azul a frente é +X; na vermelha é −X. No primeiro enable o lado trava, porque entre o autônomo e o teleoperado o robô também fica desabilitado e pode ter cruzado o meio do campo. Y vale sobre tudo isso, e as tags não o apagam. Start + Y e Start + A liberam o lado de novo. `Drive/Heading/DriverReference` mostra a origem: `SEM_TAGS_FRENTE_MAIS_X`, `TAGS_LADO_AZUL`, `TAGS_LADO_VERMELHO` ou `BOTAO_Y`. Sem tags antes de habilitar, a Driver Station recebe um aviso e a frente fica em +X.

**Sem Pigeon.** Pose e direção seguem pelo giro medido nas rodas, com alerta de erro e `Drive/Heading/YawSource = RODAS`. O erro cresce com o tempo. A visão fica parada e os resets de heading são recusados até o Pigeon voltar.

O software não consegue comprovar o alinhamento físico ao +X ao apertar Start + Y. Esse alinhamento faz parte do teste. No referencial azul usado pelo MegaTag2, zero aponta para a parede da aliança vermelha. O yaw traseiro de 180 graus da câmera é uma configuração distinta.

## Arquivos e responsabilidades

- `RobotContainer.java`: botões, condição de modo e limpeza dos limitadores. O joystick padrão só comanda movimento no teleoperado.
- `Drive.java`: freio em X, reset no estimador, referência do piloto, validação de gyro e logs. `getRotation()` retorna heading de campo; `driveFieldRelative()` usa campo; `driveDriverRelative()` usa a referência do piloto.
- `GyroIOPigeon2.java`: atualização conjunta dos sinais de yaw e velocidade angular, status de comunicação, idade dos sinais e resultado da configuração. Frequência solicitada: 50 Hz; idade máxima usada para considerar os sinais atuais: 100 ms.
- `GyroIO.java` e `GyroIOSim.java`: entradas adicionais de diagnóstico e remoção do reset físico que tinha comportamentos diferentes no real e no simulado.
- `build.gradle`: compilação dos fontes em UTF-8 para preservar comentários e mensagens em português.
- `src/test/java/frc/robot/subsystems/drive/DriveControlsTest.java`: testes de regressão sem motores reais.

O valor existente de ajuste do Pigeon (`GyroScalarZ = -5.48`) foi preservado. Uma falha ao aplicar a configuração gera aviso na Driver Station e `ConfigurationOk = false`. Esse indicador é separado de conexão: receber sinais não comprova que todas as configurações foram aplicadas.

Quando o gyro está desconectado, antigo ou numericamente inválido, os resets são recusados. A velocidade angular exposta à visão fica inválida, acionando a rejeição que já existe em `Vision`. A odometria fica sem atualização durante a falha; não foi implementada uma estimativa alternativa sem gyro nesta etapa. Uma perda do gyro durante deslocamento exige revalidar a pose após recuperar a conexão.

O seed de visão em `Drive` foi ajustado para preservar o offset de heading do campo ao redefinir X/Y. Isso é necessário para não desfazer o novo reset. Os pesos e filtros da câmera permanecem para a etapa didática seguinte.

## Primeiro teste físico: freio

1. Compilar e enviar a versão ao robô quando estiver preparado para o teste. Nenhum deploy foi feito pelo assistente.
2. Com espaço livre e analógicos centralizados, habilitar o teleoperado.
3. Segurar seta para baixo, sem Start: conferir visualmente as quatro rodas em X.
4. Conferir `Drive/XLockActive = true`, velocidades solicitadas zeradas e medidas próximas de zero.
5. Soltar: `Drive/XLockActive = false`; com joystick centralizado, o robô deve continuar parado.
6. Depois, repetir em deslocamento lento. Disable continua sendo a forma de desabilitar o robô.

No AdvantageScope, comparar `Drive/SwerveStates` e `Drive/ActualSwerveStates`. A otimização pode escolher ângulos equivalentes separados por 180 graus; isso mantém a mesma diagonal. A aprovação física depende também dos offsets e sentidos dos módulos estarem corretos.

## Segundo teste físico: yaw

1. Desabilitar e parar o robô. Conferir `Drive/Heading/GyroHealthy = true` e `Drive/Gyro/ConfigurationOk = true`.
2. Apontar a frente do robô para +X do campo azul. Pressionar Start + Y.
3. Conferir `Drive/Heading/FieldResetResult = ACCEPTED`, `FieldYawDegrees` próximo de zero e ausência de retorno ao ângulo antigo nos segundos seguintes. X/Y não são zerados por esse comando; a fusão visual pode continuar atualizando X/Y separadamente.
4. Girar manualmente o robô em um ângulo conhecido no sentido anti-horário. `FieldYawDegrees` deve aumentar de acordo com o giro.
5. Parar e pressionar somente Y. `DriverYawDegrees` e o contador manual devem zerar. `FieldYawDegrees` deve manter a direção do campo.
6. Caso um reset seja recusado, verificar o resultado e a saúde do gyro antes de repetir. Não ajustar filtros de visão para esconder falha de referência angular.

Logs de reset mostram o resultado do último pedido executado: `NOT_REQUESTED`, `ACCEPTED`, `REJECTED_GYRO`, `REJECTED_MOVING`; o reset de campo também possui proteção `REJECTED_ENABLED` se chamado diretamente. Pelo botão, Start + Y só é acionado quando desabilitado.

## Sinais para montar no AdvantageScope

| Sinal | Uso |
| --- | --- |
| `Drive/XLockActive` | Estado do comando de freio, não confirmação de alinhamento físico |
| `Drive/Heading/RawYawDegrees` | Leitura angular bruta, representada por Rotation2d |
| `Drive/Heading/FieldYawDegrees` | Direção de campo, usada pela visão |
| `Drive/Heading/DriverYawDegrees` | Direção relativa ao zero do piloto |
| `Drive/Heading/GyroHealthy` | Conexão atual e valores angulares finitos |
| `Drive/Heading/FieldResetResult` | Último resultado do reset de campo |
| `Drive/Heading/DriverResetResult` | Último resultado do reset do piloto |
| `Drive/Gyro/Connected` | Comunicação e idade dos sinais |
| `Drive/Gyro/Status` | Status retornado pela atualização CTRE |
| `Drive/Gyro/ConfigurationOk` | Resultado da configuração aplicada na inicialização |
| `Drive/Gyro/YawSignalAgeSeconds` | Idade da leitura angular |
| `Drive/Gyro/YawVelocitySignalAgeSeconds` | Idade da leitura de velocidade angular |
| `Drive/GyroManual/AccumulatedDegrees` | Giro acumulado desde o último reset do contador |

As saídas podem aparecer sob `RealOutputs` no arquivo do AdvantageKit. Os valores de entrada do gyro aparecem no grupo `Drive/Gyro`.

## Validação executada

`gradlew test jar --offline --no-daemon`: compilação e geração do programa concluídas; 11 testes passaram. Cobertura: diagonais do freio, interrupção de calibração, prioridade do comando, cancelamento em Disabled, persistência do zero, preservação de X/Y, separação piloto/campo, seed após reset, recusa de reset em movimento/habilitado, desconexão e amostra inválida do gyro.

Esses testes usam entradas controladas e o agendador WPILib. Não comprovam alinhamento físico, precisão do Pigeon, entrega CAN no robô ou calibração da câmera. Não foi realizado deploy.

## Referências

- [AdvantageKit 26.0.2: freio e pose no Drive](https://raw.githubusercontent.com/Mechanical-Advantage/AdvantageKit/v26.0.2/template_projects/sources/talonfx_swerve/src/main/java/frc/robot/subsystems/drive/Drive.java).
- [AdvantageKit 26.0.2: botão que redefine orientação pelo estimador](https://raw.githubusercontent.com/Mechanical-Advantage/AdvantageKit/v26.0.2/template_projects/sources/talonfx_swerve/src/main/java/frc/robot/RobotContainer.java).
- [WPILib 2026.2.1: cinemática e retenção dos ângulos com velocidade zero](https://raw.githubusercontent.com/wpilibsuite/allwpilib/v2026.2.1/wpimath/src/main/java/edu/wpi/first/math/kinematics/SwerveDriveKinematics.java).
- [Limelight: orientação exigida pelo MegaTag2](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2).
- APIs CTRE verificadas no arquivo de fontes instalado `wpiapi-java-26.3.0-sources.jar`, incluindo `BaseStatusSignal` e `Timestamp`.
