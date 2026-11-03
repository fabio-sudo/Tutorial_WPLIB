# Robô físico 9460 — comparação com ProjetoFrc

Análise em 28/10/2026. Escopo: código atual, projeto antigo informado pela equipe e registros locais da Driver Station de 28/10. Nenhum parâmetro de movimento, programa do robô ou configuração da simulação foi alterado nesta análise.

## Conclusão

A diferença mais relevante para a mira é a inicialização da orientação de campo: o antigo inicializa X/Y e orientação com MegaTag1, depois usa MegaTag2; o atual lê MegaTag2 e preserva a orientação do giroscópio. Além disso, o comando antigo do botão B girava o chassi para ajudar a mirar; o atual não faz esse giro. São diferenças reais de comportamento, embora os registros disponíveis não permitam atribuir o erro de um tiro específico a uma delas.

Os registros confirmam problemas de infraestrutura que precisam ser resolvidos antes da calibração do tiro: falha de gravação, endereço incorreto no AdvantageScope, alto uso de CPU/CAN, dispositivo de bancada sem comunicação e reconexões da Limelight. A curva nominal de RPM e o limite de abertura da intake permanecem iguais aos do código antigo. Não há base para reduzir RPM ou ampliar curso por tentativa cega.

## 1. Evidências dos registros físicos

Foram decodificados os sete pares `.dslog`/`.dsevents` de 28/10 em `C:/Users/Public/Documents/FRC/Log Files/DSLogs`. Os horários abaixo identificam os nomes dos arquivos; não houve conversão de fuso. Os registros iniciais contêm nomes de subsistemas de outra versão, portanto não se deve tratar todas as sessões como execução do mesmo programa.

As três sessões recentes registram classes do projeto atual, incluindo `Telescopio_9458`, `Shooter`, `Intake` e `ParallelModel` por meio de seus subsistemas. Estatísticas abaixo usam amostras indicadas como robô habilitado, com tensão maior que zero, conforme os campos da Driver Station.

| Arquivo iniciado em | Tempo habilitado registrado | CPU mediana / pico | CAN mediana / pico | Tensão mínima registrada |
|---|---:|---:|---:|---:|
| 20_33_02 | 117,30 s | 82% / 97% | 80% / 100% | 10,26 V |
| 20_35_19 | 103,48 s | 84% / 97% | 80% / 100% | 9,93 V |
| 20_37_12 | 206,38 s | 84% / 97% | 80,5% / 100% | 8,84 V |

Não há brownout indicado nessas amostras habilitadas. Isso não exclui outros problemas elétricos. A informação da DS pode repetir valores durante perdas de comunicação; estes números descrevem o registro, não uma medição elétrica independente.

### Gravação

Nas três últimas partidas, aproximadamente 0,68–0,77 s após o início do programa, aparece `Failed to open output log file` do `WPILOGWriter`. Os destinos anunciados foram:

- `/U/logs/akit_7ab812b0878a0093.wpilog`;
- `/U/logs/akit_c053a0bc23e33a22.wpilog`;
- `/U/logs/akit_ce1fa5c8c0895642.wpilog`.

A mensagem anterior “Logging to” não prova gravação bem-sucedida: vem seguida da falha de abertura. Os eventos não discriminam entre pendrive ausente, montagem, permissão, erro do dispositivo ou falta de espaço nesse destino.

Separadamente, o Phoenix relata espaço disponível de apenas 4–5 MB e recusa iniciar seu Signal Logger. A DS informa cerca de 166–170 MB livres no roboRIO. Não confundir os dois destinos/medidores nem afirmar que o pendrive está cheio apenas com essas mensagens. Verificar montagem e espaço de `/U` e do armazenamento interno antes de escolher o destino. Não habilitar gravação irrestrita no armazenamento interno como solução automática.

Não foi encontrado um WPILOG local do teste físico de hoje contendo RPM e posição dos mecanismos. Os arquivos recentes de `logs/simulacao` são simulações. Os `.dsevents` fornecem erros e eventos, mas não substituem as séries de RPM, alvo, posição e corrente. Assim, não é possível medir o RPM real do tiro, o erro angular da mira ou a posição final da intake deste ensaio.

### CAN e tempo de ciclo

O programa tenta criar o TalonFX CAN 5 do telescópio de bancada 9458 em qualquer execução real (`Telescopio_9458.createIO`, linhas 213–214). As três últimas sessões registram leituras vencidas, falha de configuração e falha de consulta de firmware desse dispositivo. Isso não comprova firmware errado: dispositivo ausente, desligado ou em outro barramento também explica as mensagens.

Há também timeouts dos Spark Flex 31 (rolete da intake) e 30 (tower) na sessão 20_33_02. O principal volume de mensagens CTRE é do CAN 5. Não há evidência para afirmar que os motores 32/34/35 estavam defeituosos.

Na sessão 20_37_12, 2.098 registros de eventos contêm `CommandScheduler loop overrun` e 288 contêm `Loop time of`. Um registro pode agrupar mensagens e o mesmo aviso pode aparecer em duas formas: estas contagens não são quantidades de ciclos únicos. Entre os avisos de todas as sessões, há `robotPeriodic()` chegando a 0,553477 s, acima do orçamento nominal de 0,020 s. Esses tempos foram registrados justamente nos avisos de atraso, portanto sua mediana não representa todos os ciclos do robô.

Recomendação: separar o perfil físico 9460 do perfil de bancada 9458, sem instanciar o motor 5 quando ele não estiver instalado. Manter esse mecanismo no perfil de bancada e na simulação. Depois medir novamente CPU, CAN e tempo de ciclo; não atribuir todo o atraso ao CAN 5 sem essa comparação.

Os modelos físicos paralelos também rodam no roboRIO, incluindo integração da intake em passos de 1 ms. São úteis, mas devem ter seu custo medido. Uma opção é executar o modelo comparativo no computador/replay ou em uma cadência apropriada no modo REAL, mantendo a simulação completa. Não basta pular ciclos sem corrigir o intervalo usado na integração.

### Limelight

O nome `limelight-rear` e o endereço `10.94.60.11` aparecem conectados ao NT4. Na última sessão, há conexões seguidas de fechamento remoto repetidas aproximadamente a cada seis segundos durante parte do ensaio. Isso confirma instabilidade de sessão; não confirma sozinho falha de alimentação, pipeline ou firmware. Verificar logs da câmera, alimentação, cabo Ethernet, endereços duplicados e configuração do servidor NT.

## 2. AdvantageScope

O arquivo local `C:/Users/Admin/AppData/Roaming/AdvantageScope/prefs.json` contém:

- `robotAddress`: **10.90.60.2**;
- `liveMode`: `nt4-akit`;
- `remotePath`: `/U/logs`.

O projeto está configurado para equipe **9460**. Para a rede padrão dessa equipe, o endereço do roboRIO é **10.94.60.2**. Os eventos mostram clientes na rede 10.94.60.x, reforçando a inconsistência do endereço salvo. O modo `nt4-akit` está adequado: `Robot.java` já cria `NT4Publisher` no modo REAL.

No AdvantageScope: `App > Show Preferences` → Robot Address `10.94.60.2`; manter NetworkTables 4 (AdvantageKit); depois `File > Connect to Robot`. Com USB direto ao roboRIO, usar a opção de endereço USB (`172.22.11.2`). Para simulação, `Connect to Simulator` continua usando `127.0.0.1`.

A conexão ao vivo e a gravação em pendrive são problemas independentes. Corrigir o IP não recupera os arquivos cuja abertura falhou. As preferências externas não foram modificadas nesta análise.

## 3. Mira: diferenças importantes para o código antigo

### Orientação inicial de campo

Antigo: `ProjetoFrc/.../CommandSwerveDrivetrain.java`, linhas 380–416, usa MegaTag1 na inicialização, com critérios de confiança, faz `resetPose` completo e aguarda transição antes do MegaTag2.

Atual: `VisionIOLimelight` só fornece MegaTag2. `Drive.tryAddVisionMeasurement`, linha 903, preserva a rotação do estimador; `seedPoseFromVision`, linha 825, também preserva essa rotação. O comando de sincronização manual de X/Y não corrige um heading inicial errado.

O MegaTag2 exige orientação de campo conhecida. Se o robô ligar em uma direção diferente da referência assumida, a leitura pode ser aceita sem que isso torne a orientação correta. Para corrigir: criar inicialização explícita e validada do heading no REAL, com robô parado/desabilitado, por pose conhecida ou MegaTag1 confiável. Manter o referencial azul e invalidar frames anteriores à mudança. Não aplicar reset contínuo durante condução.

### O comando antigo também girava o chassi

No antigo, o botão B chama `ShooterTrackAndShootWhileMovingCommand`, que assume o drivetrain e calcula rotação automática. No atual:

- B liga/desliga somente a mira automática do pivô;
- START+B liga/desliga a sequência de tiro;
- o comando do tiro exige shooter e centrífuga, sem assumir o drivetrain;
- se o alvo ultrapassar ±90°, informa `HUB_FORA_DO_CURSO_GIRE_O_ROBO` e bloqueia alimentação.

Portanto, reproduzir os mesmos movimentos do piloto não reproduz o comportamento antigo. A assistência de rotação pode ser adicionada como modo explícito, respeitando controle do piloto e limites da fiação. Não ampliar o curso do pivô para contornar o problema.

### Aliança, calibração e validade

O antigo usa alvo azul fixo (4,64; 4,04). O atual escolhe o hub pela aliança da DS; sem aliança, usa azul. Confirmar a aliança e o alvo físico usado no treino. Uma tag colocada fora da posição do mapa oficial produz distância de campo incorreta, mesmo detectada corretamente.

O antigo permitia ajustar orientação-base, offset e sinal da mira pelo SmartDashboard. O atual usa constantes (180°, sinal +1 e 15° por volta). Copiar os valores padrão não recupera ajustes que poderiam estar ativos no painel antigo. Confirmar centro mecânico, sentido e redução no robô antes de alterar ganhos.

A condição atual `isPoseTrusted()` exige somente uma correção desde o último reset. Depois da primeira, a pose continua sendo considerada confiável mesmo após longa perda de visão. A odometria pode sustentar perdas curtas, mas a liberação do tiro deve considerar idade/qualidade da pose e saúde do gyro, além de apenas “já viu tag”. O Drive já registra idade e correção recente.

## 4. RPM e liberação do tiro

A Limelight estima geometria/posição; o RPM real é validado pelos encoders. É necessário distinguir **alvo de RPM alto por distância errada** de **motor acima do RPM pedido**.

A curva de distância e seus fatores são iguais nos dois projetos. Valores calculados para robô parado, sem compensação de movimento (não são medições do teste):

| Distância ao alvo | Top | Tower | Centrífuga |
|---|---:|---:|---:|
| 1,10 m | 2.068 RPM | 2.248 RPM | 2.585 RPM |
| 2,00 m | 2.192 RPM | 2.372 RPM | 2.728 RPM |
| 3,00 m | 2.446 RPM | 2.626 RPM | 3.020 RPM |
| 4,00 m | 2.821 RPM | 3.001 RPM | 3.451 RPM |
| 5,20 m | 3.432 RPM | 3.612 RPM | 3.800 RPM |

O Top nominal é 3.432 RPM no extremo, pois 3.900 é multiplicado por 0,88. A tower pode ultrapassar a constante-base de 2.800 devido à regra Top+180. Confirmar a qual mecanismo se refere a impressão de RPM excessivo.

Melhorias prioritárias de software:

1. Retirar a autorização de alimentar apenas por tempo no perfil físico: hoje, após 0,75 s desde o início da preparação, `topAtSpeed || timeFallback` e `towerAtSpeed || timeFallback` permitem prontidão fora da tolerância. As rodas só começam após 0,25 s, deixando cerca de 0,50 s de aceleração antes do fallback. Essa lógica veio do antigo, mas não valida RPM.
2. Exigir os dois encoders do Top e o da tower válidos, atuais e dentro da tolerância por uma janela de estabilidade. Timeout deve informar falha, não substituir medição.
3. Rejeitar NaN/Infinity explicitamente: em `Shooter.atSpeed`, `abs(NaN) > tolerância` é falso e pode indevidamente permitir o resultado “pronto” com conexão marcada verdadeira.
4. Exigir conexão/leitura válida do pivô para “no alvo”. Hoje a prontidão testa zero e erro, sem testar `connected`.
5. Interromper a alimentação imediatamente em falha de leitura/zero/pose ou alvo fora do curso; reservar a tolerância temporal de 80 ms para oscilações benignas de RPM. A condição atual mantém alimentação nessa janela mesmo quando outras condições deixam de ser verdadeiras.
6. Rever a tower: ela recebe RPM antes de a mira ficar pronta. Se já houver bola no alimentador, desligar apenas a centrífuga pode não impedir lançamento. Validar a sequência com a geometria e presença de bola reais.

O antigo comparava módulos dos RPMs; o atual compara valores assinados. Verificar o sinal efetivamente retornado por cada encoder; não trocar para valor absoluto sem conferir o sentido físico, pois isso pode esconder inversão errada.

Os ganhos Spark e o kV inicial também foram mantidos. O kV usa 12/5676, embora o modelo do Top represente NEO Vortex. Se forem Vortex reais, validar o feedforward com medição. Isso não é evidência de que o ganho causou o problema deste teste.

Para calibrar a curva: confirmar distância com trena e visão; registrar alvo e RPM individual estabilizado, tensão, corrente, queda ao lançar e resultado do tiro em distâncias conhecidas. Ajustar uma tabela física medida, mantendo os parâmetros atuais de simulação como referência até validação. Evitar “corrigir” a física virtual para esconder erro de mira do robô.

## 5. Intake começa fechada e faltou descer

Ambos os projetos usam fechado = 0, aberto = −18 voltas e buffer de abertura = 0,10 volta. O software interrompe abertura ao observar aproximadamente −17,9 voltas; o Talon mantém limite de −18. O valor final exato depende da velocidade, amostragem, inércia e hold. A tolerância de 0,40 volta serve para indicar estado; não reduz o curso por si só.

Começar fisicamente fechada é compatível com a referência atual. Atenção: o encoder é zerado quando o programa instancia o hardware, inclusive após restart/redeploy. Se isso ocorrer com a intake aberta, a nova referência estará errada mesmo sem desligar a bateria. Os logs mostram reinícios do programa, mas não revelam a posição da intake nesse instante.

Para a próxima medição:

- Registrar posição do líder quando a abertura para, corrente/tensão, comando ativo e soft limit.
- Se parou antes de −17,9 sem limite, investigar botão, comando interrompido, zero inválido, energia ou mecânica.
- Se parou no limite e ainda falta descer, medir o incremento e a folga antes do batente; só então definir novo alvo operacional e limite físico com margem.

Não mudar simplesmente `OPEN_ROTATIONS`: hoje ele também determina `DEGREES_PER_MOTOR_ROTATION` e a redução da simulação. Isso altera o desenho e a física quando se pretendia ajustar somente a abertura real. Separar redução física medida, offset angular de montagem, alvo operacional e limites mecânicos.

A proteção pode melhorar monitorando também corrente, conexão e reset do seguidor CAN 35. O código atual monitora os sinais e reset do líder; não detecta explicitamente perda de seguimento do segundo motor. Também deve conferir o resultado de configurar o follower e limitar o alvo de hold ao curso válido. Um fim de curso físico de referência, no futuro, melhora a recuperação do zero.

## 6. Ordem recomendada sem prejudicar a simulação

1. Corrigir o endereço de conexão e verificar gravação real em `/U/logs`, baixando um arquivo não vazio e abrindo-o no AdvantageScope.
2. Selecionar explicitamente hardware 9460 ou bancada 9458; retirar consultas a dispositivos ausentes no REAL.
3. Estabilizar NT/CAN e medir custo de modelos/telemetria. Preservar o modo SIM completo e o replay.
4. Validar heading inicial, aliança, posição real das tags, montagem da câmera, centro e sentido do pivô.
5. Implementar critérios de prontidão medidos e diagnósticos de bloqueio; testar perda de câmera, leitura vencida, NaN, reset e alimentação com bola já na tower.
6. Medir curso adicional da intake e separar calibração mecânica de limites operacionais.
7. Só com os sinais válidos, calibrar RPM por distância e validar primeiro tiro parado; depois compensação em movimento e eventual assistência de rotação do chassi.

Não foram alterados ganhos, limites, perfis, cálculos de tiro, IOs ou modelos durante esta revisão. Não houve deploy ou acionamento do robô.

## 7. Verificação e rastreabilidade

`gradlew check --offline --console=plain`: BUILD SUCCESSFUL, tarefas UP-TO-DATE. Os relatórios existentes contêm 188 testes, zero falhas/erros/skips, incluindo simulação, mecanismos, visão e replay. O Gradle reutilizou os resultados; não houve uma nova execução dos testes nesta análise. Testes de desktop não certificam montagem, rede ou desempenho do roboRIO.

Os decodificadores locais verificaram versão e limites dos registros e consumiram integralmente os sete arquivos de cada formato. Evidências completas e resumos estão em `build/analise-fisico-20261028`: `dslog_resumo.json`, `contagens.json`, `eventos_resumo.json` e eventos por sessão. Contagens de mensagens são por registro, não ciclos únicos.

Fontes primárias:

- [AdvantageScope — conexão ao vivo](https://docs.advantagescope.org/overview/live-sources/).
- [Limelight — orientação de campo e MegaTag2](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2).
- [AdvantageScope — leitor DSLog](https://github.com/Mechanical-Advantage/AdvantageScope/blob/main/src/hub/dataSources/dslog/DSLogReader.ts) e [leitor DSEvents](https://github.com/Mechanical-Advantage/AdvantageScope/blob/main/src/hub/dataSources/dslog/DSEventsReader.ts).

Código antigo consultado em `C:/Users/Admin/Desktop/ProjetoFrc/src/main/java/frc/robot`; código atual em `src/main/java/frc/robot`. As conclusões sobre comportamento do código são revisão estática; as falhas de infraestrutura acima são evidências dos arquivos físicos encontrados.
