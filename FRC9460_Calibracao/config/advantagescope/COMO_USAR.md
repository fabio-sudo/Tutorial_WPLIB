# Visão no AdvantageScope — FRC9460

Layout preparado para AdvantageScope 26.0.0, em 09/10/2026.
As sete abas de visão foram acrescentadas às abas anteriores do usuário.

## Arquivos

- `FRC9460-Visao.json`: layout com as novas abas e as anteriores.
- `Layout-anterior-2026-10-09.json`: cópia do layout anterior.
- `preparar_layout.py`: gerador do layout, baseado no estado salvo do AdvantageScope deste computador.

Para carregar ou restaurar um layout, use **File > Import Layout...**.
A importação recria as janelas; depois reconecte a fonte dos dados.
O layout guarda gráficos e campos, não os dados de uma sessão.

## Conexão

- **Simulação local:** inicie a simulação do projeto e use **File > Connect to Simulator > NetworkTables 4 (AdvantageKit)**. O endereço mostrado será `127.0.0.1`.
- **Robô real:** com o computador conectado à rede do robô, use **File > Connect to Robot > NetworkTables 4 (AdvantageKit)**. O endereço configurado é `10.94.60.2`.
- **Log salvo:** use **File > Open Log(s)...** e selecione um `.wpilog`.

Essas fontes são suportadas pelo [AdvantageScope](https://docs.advantagescope.org/overview/live-sources/). Abrir um log permite analisar os dados registrados. Reexecutar o código com esses dados pelo replay do AdvantageKit é outra etapa: o `Robot.java` atual ainda não configura uma fonte de replay.

## Abas preparadas

Nas séries e poses de visão, **verde = Rear/traseira** e **laranja = Front/frontal**. No campo 2D, **azul = pose estimada pelo robô**.

| Aba | O que observar |
| --- | --- |
| VISAO 1 - Deteccao | Quantidade de tags, conexão e presença de alvo de cada câmera. |
| VISAO 2 - Fusao | Contador de medições aceitas, validade, novos frames e aceitação por ciclo. |
| VISAO 3 - Resultado | Resultado da fusão, contadores de rejeição, câmeras configuradas e frames pendentes. |
| VISAO 4 - Tempo | Idade da captura, limite de idade, rejeição por medição antiga/futura e saúde do giroscópio. |
| VISAO 5 - Confianca | Desvio configurado para X/Y, diferença entre poses, discordância entre câmeras e ausência de tags. |
| VISAO 6 - Campo 2D | Sobreposição da pose do robô e das poses informadas pelas duas câmeras. |
| VISAO 7 - Campo 3D | A mesma comparação usando o modelo visual genérico 2026 KitBot. |

Os campos estão ligados a `/RealOutputs/Vision/Rear`, `/RealOutputs/Vision/Front` e `/RealOutputs/Robot/Pose`. Um log antigo que não contenha esses campos não preencherá todas as abas.

## O que a câmera virtual faz hoje

`VisionIOSim` aceita frames publicados pelo método `publishFrame(...)`. Um cenário de teste fornece pose, quantidade de tags, ID, distância, timestamp e latência. O subsistema Vision valida essas entradas e pode enviá-las ao estimador do Drive.

Ela **ainda não calcula quais AprilTags estão no campo de visão da câmera**, nem gera vídeo ou simula iluminação. As tags desenhadas no campo 3D são elementos visuais; passar perto delas não gera medições automaticamente.

Na simulação normal atual, nenhum cenário chama `publishFrame`. Portanto, é esperado:

- Rear `Camera/Connected = true`;
- Rear `TagCount = 0` e `HasTarget = false`;
- nenhuma medição de visão aceita e nenhuma pose Rear nova para desenhar;
- a pose do robô mudar quando o Drive simulado se movimenta;
- campos Front sem dados, pois `FRONT_CAMERA_ENABLED` está `false`.

Sem uma captura disponível, interpretar `CaptureAgeSeconds` isoladamente não faz sentido: confira também `CaptureTimestampAvailable` e `Valid`.

O próximo passo de software é criar um cenário exclusivo de simulação que conserve as referências aos IOs e publique frames com relógio coerente com o Drive. Podemos então exercitar Rear sozinha, Front sozinha, ambas, perda de alvo, atraso e discordância. Isso testa a lógica; precisão e alcance exigem as câmeras reais.

## Primeiro teste físico, quando o robô estiver disponível

**Atualização de 14/10/2026:** os testes físicos adicionais estão adiados enquanto o usuário estiver sem o robô. A comparação de cada tag individualmente e a validação com duas câmeras reais permanecem pendentes. O roteiro de trabalho no computador está na seção abaixo.

Faça este teste com o robô parado e desabilitado, com referência de orientação coerente e tags conhecidas visíveis. Não exige pilotar nem executar autônomo.

1. Deixe a câmera vendo as tags por 5 segundos. Na aba 1, confira tags e alvo. Na aba 2, confira se `AcceptedCount` cresce. Se não crescer, confira validade, tempo e resultado da fusão.
2. Cubra a lente por 5 segundos. `HasTarget` deve ficar falso e `TagCount` deve ir para zero quando a câmera deixar de detectar. O contador de aceitação deve parar após esvaziar eventuais frames já na fila. A conexão pode continuar verdadeira: cobrir a lente não desconecta a câmera.
3. Descubra a lente e observe por mais 5 segundos. Novos frames válidos devem voltar a ser aceitos.

O teste com ambas as câmeras fica para depois de medir/configurar a montagem da frontal e habilitá-la no projeto. Então cubra uma lente de cada vez e confira que a outra continua contribuindo. Verificar somente a traseira não valida a frontal.

## Trabalho no computador sem o robô — roteiro proposto

Estas etapas ainda não estão implementadas. A revisão de 14/10/2026 confirmou que existem `ModuleIOSim`, `GyroIOSim` e `VisionIOSim`; o `Robot.java` publica a simulação por NetworkTables, mas ainda não configura replay nem grava `.wpilog` no modo de simulação. A frontal está desabilitada na configuração padrão atual.

### 1. Cenários controlados de visão

Primeira etapa recomendada: conservar referências a dois `VisionIOSim` e fornecer frames por um controlador de cenários exclusivo de SIM. Usar uma pose de referência conhecida e independente da pose corrigida pelo estimador, com um relógio comum e captura/entrega separadas para representar atraso.

Começar com o robô parado e dois sensores virtuais; depois ampliar para movimento. Os cenários devem permitir:

- Rear sozinha, Front sozinha e ambas contribuindo.
- Duas câmeras configuradas, desconectando uma de cada vez.
- Uma ou duas tags, ausência de tags e recuperação.
- Frames congelados, repetidos, antigos, futuros e entregues fora de ordem.
- Medições contraditórias entre câmeras e recuperação posterior.
- Deslocamento conhecido numa leitura, incluindo aproximadamente 8,5 cm para investigar o comportamento observado no CSV. Esse valor é uma hipótese de teste, não uma correção de calibração.

No AdvantageScope, sobrepor a referência do cenário, a pose estimada do robô e as medições das duas câmeras. Mostrar também erro em relação à referência, cenário ativo, contadores de aceitação e motivos de rejeição. Gravar as entradas simuladas em `.wpilog` para repetir o experimento.

Critérios iniciais: a câmera saudável continua contribuindo quando a outra desconecta; frames repetidos não são aplicados novamente; medições fora dos limites de tempo são rejeitadas; a fusão retoma com novas leituras válidas. Já existem testes automatizados dessas situações em `VisionMultiCameraTest`; o cenário acrescentará a observação interativa no AdvantageScope.

### 2. Visibilidade geométrica das tags

Depois dos cenários controlados, adicionar a pose de montagem de cada câmera, campo de visão e mapa de AprilTags. O objetivo é calcular quais tags são visíveis conforme o robô virtual se movimenta. O [template oficial de visão do AdvantageKit](https://docs.advantagekit.org/getting-started/template-projects/vision-template/) usa PhotonLib para simulação de visão e também suporta Limelight real.

Essa opção não exige trocar as câmeras físicas. A simulação modela observações; não reproduz o firmware MegaTag2 nem valida iluminação, impressão das tags ou a calibração óptica real. As constantes provisórias da frontal podem servir como parâmetros virtuais, identificadas como tal.

### 3. Replay de dados gravados

Configurar explicitamente os modos REAL, SIM e REPLAY. No replay, usar `WPILOGReader` e escrever um novo log para comparar os resultados originais e recalculados, conforme a [documentação do AdvantageKit](https://docs.advantagekit.org/getting-started/traditional-replay/).

O replay exige as entradas originais registradas. Os CSVs analisados têm campos de entrada ausentes e não bastam para reconstruir fielmente a execução. Não foi encontrado `.wpilog` nesta pasta do projeto durante a revisão. Podemos começar gravando um log do próprio cenário de simulação e depois usar logs completos do robô quando estiverem disponíveis.

No replay, mudanças de código não alteram os sensores já gravados; testes em que o movimento precisa reagir a novos comandos pertencem à simulação interativa.

## Como interpretar sem confundir os sinais

- `NewMeasurement` e `MeasurementAccepted` são sinais por ciclo: podem alternar entre verdadeiro e falso. Use também `AcceptedCount` para acompanhar o progresso.
- `Valid` indica que a leitura passou nas verificações iniciais; não significa que uma medição já saiu da fila e foi aplicada ao Drive.
- `FusionResult = REPEATED_FRAME` indica releitura do mesmo frame; `QUEUED` indica espera na fila; `INVALID_FRAME` exige consultar os motivos em `Reject`.
- O valor de `RejectedCount` acompanha rejeições contabilizadas pelo processamento da fila/fusão; não é uma contagem completa de todos os frames inválidos na entrada.
- Uma pose de câmera pode continuar desenhada com o último valor mesmo após perder as tags. Confira `Valid`, `HasTarget` e o tempo antes de interpretá-la como leitura atual.
- `XYStdDevMeters` é o parâmetro de confiança passado ao estimador, não uma medição da precisão real. `PoseErrorMeters` também não substitui uma referência física medida.
- O robô 3D é apenas a representação visual escolhida. Não é o modelo mecânico exato do swerve nem a imagem que a Limelight enxerga.

No código atual, o modo real configura gravação WPILOG e publicação ao vivo. A simulação configura publicação ao vivo; não foi acrescentada gravação WPILOG à simulação nesta configuração de layout.
