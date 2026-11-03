# Validação de movimento, visão, disparo e replay

Revisão: 30/10/2026. Este roteiro compara o pedido do código, os sensores e,
quando disponível, uma referência independente do resultado.

**Guia visual do ciclo completo** (ligar o robô, testar, baixar o log, gerar o replay
e ajustar): [GUIA_TESTE_E_REPLAY.png](GUIA_TESTE_E_REPLAY.png), gerado a partir de
`guia_teste_e_replay.html`.

## Arquivos para abrir

| Aplicativo | Arquivo | Uso |
|---|---|---|
| AdvantageScope | `config/advantagescope/layouts/FRC9460-ValidacaoRobo.json` | 18 abas: campo, mecanismos, liberação do tiro, visão, movimento, mira, RPM, tiro, simulação, chegada, gravação, intake (segurar a posição) e heading pelas tags |
| AdvantageScope | `config/advantagescope/layouts/FRC9460-Replay.json` | 6 abas: pose, movimento pedido, mira, RPM, decisões e previsão do tiro; original × recalculado |
| AdvantageScope + Elastic | `FRC9460-ConsistenciaDeTiro.json` | Constância de velocidade, mira e acerto entre os tiros, com o 3D de onde chegou × onde devia. Guia: [CONSTANCIA_DE_TIRO.md](CONSTANCIA_DE_TIRO.md) |
| AdvantageScope + Elastic | `FRC9460-PoseLimelight.json` | Pose estimada, Limelight, inicialização pelas tags e heading. Guia: [POSE_E_LIMELIGHT.md](POSE_E_LIMELIGHT.md) |
| AdvantageScope | `config/advantagescope/layouts/FRC9460-Campo2D3D.json` | 29 abas: 11 em 2D e 3D (partida, robô de perto, visão pela Limelight, tiro, pose, ciclo da bola, replay) e 18 de log, replay, tensão, real × simulado e PID |
| Elastic | `config/elastic/layouts/FRC9460-ValidacaoRobo.json` | 8 telas para acompanhar o ensaio ao vivo; grade de 1152 × 576 px sem sobreposição |
| Elastic | `config/elastic/layouts/FRC9460-Analise.json` | Painel único de análise, 5 telas: pré-jogo, campo e câmera, tiro, mecanismos e saúde do robô |

## Experiência 2D e 3D (`FRC9460-Campo2D3D.json`)

| Aba | O que mostra |
|---|---|
| 2D Partida | Pose estimada, vetores dos módulos (medido em ciano, pedido em laranja), fantasmas da visão, linhas até as tags e mapa de calor de onde o robô andou |
| 3D Partida | Campo inteiro com robô, mecanismos, visão e bolas |
| 3D Robô de perto | Câmera colada no robô: mecanismos, módulos, linha do shooter e previsão do tiro |
| 3D Limelight | O campo pelo ângulo da Limelight traseira, com as linhas até as tags usadas |
| 2D Tiro | Mapa de calor de onde atira e de onde as bolas chegam |
| 3D Tiro | Anel do hub, cones de chegada e linha de erro até o centro: planejado (laranja), sensores (ciano), simulação (vermelho) |
| 2D/3D Pose | Estimada × fantasmas MegaTag2 e MegaTag1 × tags |
| 3D Ciclo da bola | Coleta e disparo, com as bolas no chão e no ar |
| 2D/3D Replay | Robô gravado × código atual (fantasma laranja) |

Para ver 2D e 3D **lado a lado**, deixe a aba 2D aberta e use **Tabs > New Pop-Out**
(Ctrl+Shift+T): abre uma janela com a aba atual. Troque a janela principal para a
aba 3D e as duas seguem o mesmo tempo do log.

Na aba 3D Limelight, a câmera vem de `cameras` em
`mecanismos/Robot_Robo_9460/config.json`. Se mudar a Limelight de lugar, atualize
ali também: a aba não usa o `VisionConstants`.

Vetores dos módulos, linhas até as tags e cones em forma de seta para baixo só
existem nos logs gravados a partir desta versão. Em log antigo, essas fontes
ficam vazias na aba; o resto funciona.

AdvantageScope: **File > Import Layout**. Abra o `.wpilog` em **File > Open Log(s)**.
Para o modelo 3D, escolha `config/advantagescope/mecanismos` em
**App > Use Custom Assets Folder**. O campo 2D não depende do CAD personalizado.

Elastic: **File > Open Layout**. O painel também vai na pasta deploy e pode ser
baixado por **File > Download Layout** após enviar o projeto ao robô.
Conecte ao time 9460 para o físico ou a `localhost` para a simulação.
O painel é de acompanhamento; não adiciona botões para comandar motores.

## Log, tensão e PID (`FRC9460-Campo2D3D.json`)

Depois das abas 2D e 3D, o layout tem as abas de calibração. Elas funcionam em log
do robô real, em log de simulação e ao vivo.

| Abas | O que respondem |
|---|---|
| `LOG` | O log está gravando? Arquivo, espaço livre, tempo de ciclo, bateria, CAN e brownout |
| `REPLAY` | O que o código atual pediria com os mesmos sensores. Só aparece no `_replay.wpilog` |
| `MEC` | Tensão e corrente de todos os mecanismos no mesmo gráfico, e a diferença entre real e simulado |
| `PID` | Pedido, medido e simulado de cada mecanismo, com tensão, corrente e erro |

**Simulado** é a física recebendo a tensão medida no motor, então existe também no
log do robô real. Diferença pequena: o modelo está bom. Diferença que cresce devagar:
atrito, inércia ou redução diferentes. Salto: batida, travamento ou bola presa.

| O que aparece na aba `PID` | Provável causa | Onde mexer |
|---|---|---|
| Fica abaixo do alvo, com erro constante | Falta ganho ou feedforward | kP ou kV |
| Passa do alvo e volta | Ganho alto | kP menor, ou kD maior no pivô e no steer |
| Treme em volta do alvo, com a tensão alternando | kS alto ou ruído | kS menor, ou banda neutra maior |
| Tensão no limite e o medido não sobe | Limite de corrente, bateria ou mecânica | Não é ganho: veja a corrente e a bateria |
| Pivô atrás do perfil | kV baixo | Tensão ÷ velocidade no trecho constante |

| Mecanismo | Constantes | Arquivo |
|---|---|---|
| Intake | `HOLD_KP`, `HOLD_KG`, `HOLD_NEUTRAL_DEADBAND` | `IntakeConstants.java` |
| Pivô | `PIVOT_KP`, `PIVOT_KD`, `PIVOT_KS_VOLTS`, `PIVOT_KV_VOLTS_PER_ROTATION_PER_SEC`, `PIVOT_NEUTRAL_DEADBAND`, perfil | `ShooterConstants.java` |
| Top e tower | `SPARK_KP`, `SPARK_KV_VOLTS_PER_RPM` | `ShooterConstants.java` |
| Centrífuga | `KP`, `KS`, `KV` | `CentrifugaConstants.java` |
| Drive e steer | `driveGains`, `steerGains`, `steerMotionMagic` | `ModuleIOTalonFX.java` |

Mude um ganho por vez e repita o mesmo movimento. Top, tower e centrífuga mudam o
tiro: depois de ajustar, confira a curva (`CONSTANCIA_DE_TIRO.md`).

## O significado de cada comparação

| Comparação | O que demonstra | Limite |
|---|---|---|
| Movimento pedido × medido | Resposta dos módulos às velocidades solicitadas, em m/s e rad/s no referencial do robô | Encoders não medem derrapagem no chão |
| Pose estimada × visão | Coerência entre odometria e câmeras | A visão também tem erro; pose sem novas tags pode ser o último valor válido |
| Pose estimada × referência SIM | Erro do estimador frente ao mundo simulado | A referência física verdadeira não existe no robô real |
| Pivô/RPM alvo × sensores | Se o mecanismo atingiu a condição solicitada | Não comprova a trajetória real da bola |
| Tiro planejado × previsão pelos sensores | Efeito dos RPM e ângulos medidos no modelo balístico | Ambos são previsões; `Sensores/Acerta` não é detecção visual de cesta |
| Resultado SIM / anotação REAL | Resultado do voo simulado / observado pela equipe | A equipe precisa associar a anotação ao número correto do tiro |
| RealOutputs × ReplayOutputs | Decisões originais × decisões do código atual com as mesmas entradas | Alterar o comando no replay não muda os sensores gravados nem produz outra física |

Ciano representa medição/original; laranja, pedido/replay. Na visão, roxo é a
traseira e verde é a frontal. Nas trajetórias 3D, laranja é o tiro planejado,
ciano é a previsão pelos sensores e vermelho é o voo simulado.

## Intake e heading: o que procurar (abas 15 a 18 e tela 08 do Elastic)

Criadas depois do teste de 29/10/2026 (intake fazendo barulho parada, mira um
pouco torta, tiros passando do hub). As duas abas da intake e a de heading
usam campos que só existem em logs gravados depois dessa data.

| Aba | Pergunta | O que procurar |
|---|---|---|
| 15 - Intake: segurar a posição | A intake está quieta parada? | Aberta: `RetencaoEmRepouso` ligado e tensão 0 V (só Brake, braço no batente). Fechada ou no meio: `CorrecaoPosicaoAtiva` ligado com tensão **constante** de poucas dezenas de mV. Qualquer troca entre os dois com o braço parado é defeito (antes de 30/10 eram 20 a 40 por segundo). |
| 16 - Intake: real × modelo | A intake se comporta como a física? | Diferença que cresce com o braço parado = atrito, folga ou massa diferentes da estimativa de 3 kg. |
| 17 - Heading: Pigeon × tags | O heading definido à mão (Start + A) está certo? | Robô parado vendo 2 ou mais tags: `ErroHeadingGraus` é quanto o heading está errado, e a mira sai torta na mesma quantidade. NaN = sem leitura confiável. Com uma tag só o MegaTag1 pode escolher a solução espelhada. |
| 18 - Campo: pose × MegaTag1 | Onde o robô acha que está? | Fantasma vermelho = pose pelo MegaTag1; roxo = MegaTag2. Giro dos fantasmas em volta das tags = heading errado. |

O MegaTag1 é só diagnóstico: não entra no estimador.

## Ensaio automático disponível

Na raiz, com o JDK do WPILib configurado:

```powershell
.\gradlew.bat check --offline --console=plain
```

O teste integrado simula controles, movimenta o robô, coleta bolas, mira e
atira no hub, exercita o telescópio e executa BACK + A para chegar a 2 m.
Depois roda o mesmo roteiro com os sensores e controles gravados.

Resultados:

- `build/ciclo-da-bola/CicloDaBola-Teste.wpilog`: cenário simulado, não coleta física.
- `build/ciclo-da-bola/CicloDaBola-Teste_replay.wpilog`: original e saídas recalculadas.
- `build/ciclo-da-bola/validacao-resumo.json`: contagens e erro de chegada.
- `build/reports/tests/`: relatórios dos testes.

O teste exige contagens coerentes de coleta/disparo, pelo menos 10 acertos
simulados, distância longitudinal de 2 m ± 3 cm sem timeout, tópicos dos
layouts presentes e tipos corretos dos widgets do Elastic. O teste de replay
compara pedidos de translação, RPM, pivô e contagem de disparos em cada ciclo.

O erro XY é registrado separadamente no resumo: conclusão do comando não
significa estar a até 3 cm do ponto no campo. Na revisão, o ensaio revelou
cerca de 4,9 cm de erro XY, apesar de concluir a distância longitudinal.

## Movimentar manualmente na simulação

Execute `gradlew.bat simulateJava --offline --console=plain`, conecte o
controle à porta 0 e habilite **Teleoperated** na interface de simulação.
Use os analógicos para dirigir, R1/L1 para baixar/subir o intake, R2 para
puxar com o rolete, L2 para mirar e B para mirar e atirar enquanto segurado.
O mapa completo continua em `CONTROLES_E_MECANISMOS_2026-10-29.md`.

No teste BACK + A, o alvo é **2 m à frente da pose inicial**, mesmo quando o
robô começa girado. O fantasma laranja e a linha amarela mostram destino e
segmento esperado. A aba de chegada mostra erro XY, desvio lateral e heading.
O comando existente termina pela distância longitudinal; o indicador XY
revela se houve desvio lateral. Isso não constitui uma rota autônoma de jogo.
Os dados dessa aba aparecem depois de executar o teste, e ficam no último
resultado quando `Active=false`.

## Replay de um log real

Baixe o `.wpilog` do roboRIO com o robô ligado: AdvantageScope, **File >
Download Logs...**, endereço `10.94.60.2` (por USB, `172.22.11.2`), pasta
`/U/logs` (pendrive) ou `/home/lvuser/logs/advantagekit` (sem pendrive).
O painel de gravação mostra o caminho e o estado. Depois execute:

```powershell
.\config\validacao\replay_robo.ps1 -Log 'C:\caminho\FRC9460_coleta.wpilog'
```

**Use pendrive.** O log fica no robô, nunca na pasta do projeto, e o disco interno
tem pouco espaço: gravando cerca de 10 MB por minuto, ele acaba em poucas sessões.
Para saber se uma sessão gravou, procure no console da Driver Station:

- `[AdvantageKit] Logging to "..."`: gravando nesse caminho.
- `[FRC9460] Log NÃO está gravando`: esta sessão ficará sem replay. A mensagem diz
  o motivo e o espaço livre.

O script copia o log para uma pasta exclusiva em `logs/replay`, executa
o código atual e preserva tanto o original quanto os replays anteriores.
Abra o resultado `_replay.wpilog` e importe `FRC9460-Replay.json`.
As tarefas de replay já existentes no VS Code continuam disponíveis.

O modo REPLAY é escolhido para essa execução no computador. O roboRIO continua
em REAL. O replay não inicializa CAN, câmeras virtuais nem física de bolas.
O Elastic acompanha REAL/SIM ao vivo; neste projeto, o replay é inspecionado
no arquivo do AdvantageScope, sem publicação ao Elastic.

Logs antigos não ganham retroativamente entradas que não foram gravadas.
Novos campos de chegada e identificação da compilação exigem uma nova coleta.
Um `.dslog`, CSV exportado ou vídeo não substitui todos os inputs do `.wpilog`.

## Leitura durante um tiro

1. Confira o modo e a gravação. No físico, `GRAVANDO_USB` ou `GRAVANDO_INTERNO`
   deve aparecer, com `FilaComFalha=false`.
2. Confira conexão, tags e resultado da fusão. A frontal física está desativada
   em `VisionConstants`; os campos frontais podem ficar sem dados no real.
3. Leia idade da correção e limite. `PoseConfiavel` significa que a visão já
   corrigiu desde o reset; **não garante correção recente**. Os dois indicadores
   ficam separados porque o código atual só exige a primeira condição.
4. Leia `Estado` e `Detalhe` da sequência, pivô pronto, top/tower prontos,
   alcance do hub e `Alimentando`. Alimentação é comando, não confirmação de bola.
5. Localize `DisparoAgora`/número do tiro. Confira queda de RPM, visão no
   instante do disparo e previsões. Esses registros ficam congelados até o
   próximo disparo detectado.
6. No físico, anote o resultado pelos controles já existentes em
   `/Tuning/Fuel/AnotarTiroReal`: Acertou, ErrouCurto, ErrouLongo ou ErrouDoLado.
   Confira `TiroAnotado` antes de comparar com `Tiro/Numero`.

## Regenerar todos os layouts

```powershell
python config/advantagescope/scripts/preparar_layout_validacao_robo.py
```

O gerador verifica limites e sobreposição dos widgets. O teste integrado
confere os tópicos usando o log produzido pelo próprio projeto, e o
`AdvantageScopeLayoutFormatTest` confere o formato das abas 2D e 3D.

Linhas até as tags, vetores dos módulos e peças do robô são **filhos**: pertencem
à fonte logo acima. Depois de um fantasma sairiam dele, e depois de uma trajetória
não seriam desenhados. O gerador já os coloca logo abaixo do robô.

Referências: [AdvantageScope](https://docs.advantagescope.org/) e
[importação de layouts no Elastic](https://frc-elastic.gitbook.io/docs/getting-started/app-navigation).
