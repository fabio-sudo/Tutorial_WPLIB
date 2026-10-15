# Análise da Limelight traseira — 09/10/2026

Fonte: `C:/Users/Admin/Desktop/log.csv`, fornecido pelo usuário. Intervalo do registro: 508,807 a 620,294 segundos do relógio do robô. O CSV original foi somente lido.

## Resultado

O registro confirma que a câmera traseira recebe medições válidas, interrompe a aplicação após esvaziar a fila durante perdas prolongadas de tags e retoma a fusão quando as tags reaparecem. A conexão da câmera e a saúde do giroscópio permanecem verdadeiras em todas as amostras disponíveis.

Há uma câmera configurada durante todo o intervalo. Isso valida o funcionamento da traseira sozinha nesta execução. Não exercita duas câmeras configuradas com uma delas desconectada, nem a comparação entre câmeras.

## Números principais

| Indicador | Resultado |
| --- | --- |
| Duração analisada | 111,487 s |
| Linhas do CSV | 5.566 |
| Intervalo mediano entre linhas | 20,0 ms |
| Contador de medições aceitas | 20.787 → 24.661; aumento de 3.874 |
| Tempo com duas tags | Aproximadamente 78,56 s |
| Tempo com uma tag | Aproximadamente 1,76 s |
| Tempo sem tags | Aproximadamente 31,17 s |
| Idade mediana da captura, com `Valid = true` | 33,63 ms |
| Percentil 95 da idade da captura válida | 45,31 ms |
| Maior idade da captura válida | 63,12 ms |
| Limite configurado de idade | 250 ms |
| Maior fila observada | 5 medições |

Não há amostras com `StaleMeasurement`, `FutureTimestamp`, `Rotation`, `NoEstimate` ou `NoPose` verdadeiros. Em todas as linhas com tags, `Valid` é verdadeiro. Isso mostra que os filtros de entrada não bloquearam as detecções com tags neste intervalo; não significa que todos os sinais brutos puderam ser auditados.

## Perda e recuperação

Os tempos abaixo são os mesmos que podem ser selecionados no AdvantageScope.

| Intervalo sem tags | Comportamento observado |
| --- | --- |
| 508,807–527,973 s | O registro começa sem tags. Contador permanece em 20.787 e fila em zero durante 19,17 s. Tags retornam a 527,973 s e o contador volta a crescer a 528,053 s: cerca de 80 ms depois. |
| 584,194–589,074 s | Perda de 4,88 s. A fila esvazia em aproximadamente 60 ms; depois o contador permanece fixo. A aceitação retoma cerca de 80 ms após o retorno das tags. |
| 589,174–596,154 s | Nova perda de 6,98 s. A fila esvazia em aproximadamente 20 ms; depois o contador permanece fixo. Uma breve detecção volta e sua medição é aplicada cerca de 80 ms depois. |

Também há perdas breves de aproximadamente 20–80 ms. Uma medição pode ser aplicada quando o frame atual já está sem tags, porque ela foi capturada antes e aguardava na fila. Nas perdas prolongadas, não houve aumento do contador depois que a fila esvaziou.

O tempo de 80 ms é observado entre a primeira linha que volta a ter tags e a primeira aplicação subsequente. Não é a latência óptica da câmera. A resolução típica do CSV é de 20 ms; perdas muito curtas podem coexistir com a aplicação de frames anteriores.

## Diferença de orientação

Em cada período contínuo sem tags, os valores X, Y e orientação da pose da câmera ficam exatamente constantes. A pose do robô continua podendo mudar. A diferença angular entre os desenhos chega a 56,81° nessas condições.

Com `Valid = true`, 95% das diferenças angulares entre a pose atual do robô e a pose registrada da câmera ficam abaixo de 0,290°. O máximo é 5,72°, sem compensação temporal desta comparação durante o movimento.

Nos últimos 10 segundos, as velocidades de translação medidas são zero e a orientação do robô varia apenas 0,0165°. Nesse trecho, a maior diferença angular entre as duas poses é aproximadamente 0,00275°. Portanto, não aparece uma diferença persistente de dezenas de graus após a recuperação e estabilização.

Essa concordância não mede a precisão absoluta do giroscópio: o MegaTag2 usa a referência angular enviada pelo robô. O projeto aplica a visão à translação e preserva a orientação do estimador.

## Oscilação da posição

Nos últimos 10 segundos, a distância média informada às tags tem mediana de 3,14 m. Há duas tags em 499 das 500 linhas, e uma tag em uma linha.

Nesse trecho, os 90% centrais dos valores da visão ocupam uma faixa de aproximadamente 3,52 cm em X e 1,12 cm em Y. A pose estimada do robô varia, do mínimo ao máximo, 3,73 cm em X e 1,23 cm em Y.

A 615,974 s, a linha com uma tag apresenta um desvio de aproximadamente 12,1 cm em relação à mediana das posições de visão dos últimos 10 segundos. É um pico isolado nesse trecho, a observar na comparação entre uma e duas tags. Não é, por si só, justificativa para descartar toda medição de uma tag.

`TagCount` e a pose visualizada descrevem a leitura atual; `XYStdDevMeters`, `PoseErrorMeters` e `FusionResult` podem descrever uma medição anterior processada pela fila. Por isso, não se deve atribuir o peso ou o resultado da mesma linha àquele pico sem o timestamp do frame aplicado.

Concordância entre visão e estimador não estabelece precisão real: o estimador já utiliza a visão. É necessária uma posição física medida para calcular erro absoluto.

## Limites da análise e próximo passo

- As estatísticas descrevem as linhas exportadas, que podem repetir valores. Elas não são uma contagem de imagens independentes. O total aceito foi calculado pela diferença do contador, não pelo número de linhas `ACCEPTED`.
- Alguns campos, incluindo timestamps brutos de entrada e vários diagnósticos, estão inteiramente `null`. Eles foram tratados como indisponíveis, nunca como zero.
- A comparação de orientação usa a menor diferença angular, após converter radianos em graus. Não compara o ângulo atual com o histórico exatamente no instante da captura, pois os timestamps necessários não estão preenchidos.
- `RejectedCount = 0` não conta todas as leituras inválidas na entrada; as linhas sem tags têm `Valid = false`.

O próximo teste útil é manter o robô parado numa posição medida, primeiro vendo duas tags e depois somente uma, comparando a dispersão e os picos. Convém manter os filtros atuais durante essa coleta. Quando a frontal estiver disponível e configurada com a montagem correta, deve-se repetir o teste com ambas habilitadas e desconectar ou ocultar uma delas de cada vez.
