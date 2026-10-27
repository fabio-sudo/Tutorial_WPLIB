# Gravação e replay da visão

## O que foi habilitado

- **REAL:** mantém gravação no pendrive do roboRIO e publicação NT4.
- **SIM:** mantém as duas câmeras PhotonLib, publica NT4 e grava um `.wpilog`
  em `logs/simulacao`. Cada execução cria seu próprio arquivo,
  `Simulacao_aa-MM-dd_HH-mm-ss.wpilog`.
- **REPLAY:** lê módulos, gyro e câmeras do arquivo. Não executa o mundo
  PhotonLib nem cria dispositivos CAN ou sensores simulados de motores.
  A fila de visão, os filtros e o estimador do Drive são recalculados.

`LoggingConstants` guarda o modo padrão, a pasta de gravação e o sufixo de saída.
O desktop continua em SIM. No roboRIO, a execução continua sempre em REAL.
Não é preciso alterar o código para trocar o modo durante uma análise.

## Primeira gravação sem o robô

1. Encerre a execução antiga do simulador e inicie uma nova após recompilar.
2. Use **WPILib: Simulate Robot Code** com a Simulation GUI, como antes.
3. Conecte o AdvantageScope em `127.0.0.1`.
4. Movimente e gire devagar no simulador. Registre também alguns segundos parado.
5. Volte a Disabled e encerre o programa pelo ambiente de simulação.
6. Localize o arquivo mais recente em `logs/simulacao`. Use esse `.wpilog` completo,
   não a exportação CSV do AdvantageScope.

Os arquivos da pasta `logs` já são ignorados pelo Git. Não grave uma sessão
indefinidamente: encerre a execução quando terminar a coleta.

## Recalcular a mesma sessão

No terminal WPILib do projeto, execute, substituindo pelo caminho do seu arquivo:

```powershell
.\gradlew.bat simulateJava '-PreplayLog=C:/caminho/coleta.wpilog' --offline --console=plain
```

O argumento deve ser passado inteiro, entre aspas, se o caminho tiver espaços.
O Gradle seleciona REPLAY e desativa a GUI por padrão para essa execução.
Mantenha todas as extensões HAL desmarcadas no seletor do VSCode quando usar
esse modo; nem Simulation GUI nem conexão com Driver Station devem estar ativas.

O resultado fica ao lado do original, com sufixo `_replay.wpilog`, e é oferecido
ao AdvantageScope automaticamente ao terminar. O original é preservado.
Repetir a análise do mesmo original substitui seu resultado anterior com esse
sufixo. Para guardar comparações de ajustes diferentes, copie/renomeie os
resultados antes de executar novamente.

Outra opção é definir `FRC9460_MODE=REPLAY` e fornecer `AKIT_LOG_PATH`, conforme
o mecanismo do AdvantageKit. Sem caminho explícito, `LogFileUtil` tenta usar o
arquivo aberto no AdvantageScope e, por último, solicita o caminho no terminal.
Remova a seleção de modo ao voltar à simulação normal.

`replayWatch` também seleciona REPLAY e pode receber o mesmo `-PreplayLog`.
Ele permanece ativo e repete o replay quando o código muda; encerre-o ao terminar.

## Comparação no AdvantageScope

No arquivo resultante:

| Caminho | Origem |
| --- | --- |
| `/RealOutputs/Robot/Pose` | Estimativa original da sessão |
| `/ReplayOutputs/Robot/Pose` | Estimativa recalculada pelo código atual |
| `/RealOutputs/Vision/Rear/Fusion/*` | Decisões originais da traseira |
| `/ReplayOutputs/Vision/Rear/Fusion/*` | Novas decisões da traseira |
| `/ReplayOutputs/Vision/Front/Fusion/*` | Novas decisões da frontal, se gravada |
| `/Drive/Module*/`, `/Drive/Gyro/` | Sensores originais usados novamente |
| `/Vision/Rear/Inputs/`, `/Vision/Front/Inputs/` | Entradas originais das câmeras |
| `/Drive/Startup/` | Inicialização da pose simulada, gravada nas novas sessões |

No Campo 3D, use `ReplayOutputs/Robot/Pose` como **Robot** e
`RealOutputs/Robot/Pose` como **Ghost**, ambos com o modelo FRC9460.
Na simulação, adicione também `RealOutputs/Vision/Simulation/ReferencePose`
como outro Ghost para manter uma referência que não recebe correção visual.
Compare `AcceptedCount`, `FusionResult`, `XYStdDevMeters` e `Reject/*` nos
dois grupos de saídas, no mesmo instante do log.

## O que esta etapa permite validar

- Modificar pesos e filtros em `VisionConstants` e comparar a fusão usando
  exatamente as mesmas observações.
- Repetir perda de alvos, falha de uma câmera, lotes e timestamps registrados.
- Conferir a pose inicial `(2, 2, 0°)` em replay de uma nova sessão simulada.
  Ela é aplicada uma vez; não copiamos a pose fundida de `RealOutputs`.
- Reproduzir a frontal quando houver inputs dela, mesmo que sua configuração
  física esteja desativada. Se não houver frontal no log, permanece desconectada.

## Limites

Replay precisa de logs AdvantageKit com os inputs dos IOs. CSV ou um log contendo
apenas poses de saída não reconstrói os sensores. Logs antigos da simulação sem
`Drive/Startup` não restauram automaticamente a origem `(2, 2)`; prefira uma nova
gravação com este código. A inicialização física permanece como antes.

As poses das câmeras já estão calculadas no arquivo. Alterar lente, mapa ou
montagem e repetir o replay não reprocessa imagens nem recalcula MegaTag2/PhotonLib.
Para avaliar essas alterações, execute uma nova simulação ou coleta física.
Mudanças nos comandos também não alteram os movimentos dos sensores já gravados.

Esta etapa verifica software e ajuda a ajustar a fusão. A precisão física ainda
depende dos testes de montagem, gyro, tags e deslocamento medido.

## Validação automatizada

`gradlew build` inclui `visionReplayTest`, em uma JVM separada. Esse teste grava
um WPILOG de entradas controladas, lê as duas câmeras pelo Logger, inicializa a
pose, funde observações PhotonLib e MegaTag2, rejeita um frame repetido e mantém
a traseira funcionando após a frontal desconectar. O arquivo resultante é
inspecionado para conferir `RealOutputs` e `ReplayOutputs`, preservando a entrada.
Esses dados sintéticos de teste não devem ser tratados como coleta do robô.

Referências:

- [Configuração do AdvantageKit em projetos existentes](https://docs.advantagekit.org/getting-started/installation/existing-projects/)
- [Traditional Replay](https://docs.advantagekit.org/getting-started/traditional-replay/)
- [Replay Watch](https://docs.advantagekit.org/getting-started/replay-watch/)
- [Relógio e Driver Station registrados](https://docs.advantagekit.org/data-flow/built-in-logging/)
