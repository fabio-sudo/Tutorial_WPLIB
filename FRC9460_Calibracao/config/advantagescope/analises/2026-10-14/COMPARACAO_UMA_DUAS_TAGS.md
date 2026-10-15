# Comparação entre uma e duas AprilTags

Análise concluída em 14/10/2026 a partir do segundo `log.csv` enviado pelo usuário. O arquivo original estava em `C:/Users/Admin/Desktop/log.csv`. A análise usa a cópia preservada em `build/analise_comparacao_tags_20261009/log_recebido.csv`, cujo conteúdo foi conferido novamente antes da conclusão.

SHA-256 da cópia analisada: `f89ea0f38b4a757869325dd3bbb74cd383dbfe7fb59126df22dcbe4399d65039`.

## Resultado principal

A câmera traseira continuou fornecendo medições aceitas com uma tag. Entretanto, a posição típica da visão mudou aproximadamente **8,55 cm** entre o primeiro trecho com duas tags e o trecho estável com uma tag. Quando duas tags voltaram a aparecer, a posição típica retornou para aproximadamente **1,46 mm** da posição típica inicial.

Esses números comparam as medianas das leituras. Eles não representam erro em relação à posição física verdadeira, que não foi fornecida. A repetibilidade entre os dois trechos com duas tags também não demonstra precisão absoluta de milímetros.

As velocidades medidas de translação e rotação ficaram em zero durante todo o registro. O yaw informado variou apenas 0,0302°. Isso é compatível com o teste parado, mas os sensores não são uma medição externa da posição física.

## O que foi registrado

- Intervalo: **30,887–96,755 s**, duração de **65,868 s**, com 3.284 linhas.
- Duas tags: aproximadamente **51,63 s** no total.
- Uma tag: aproximadamente **9,54 s** no total; maior trecho contínuo de **7,00 s**.
- Nenhuma tag: aproximadamente **4,70 s**, distribuídos na transição.
- Uma câmera configurada, conectada e com indicação de giroscópio saudável em todas as linhas.
- Contador de medições aceitas: **851 → 3.764**, aumento de **2.913** entre os primeiros e últimos valores disponíveis.
- Todas as linhas com tags tiveram `Valid = true`.
- Idade das capturas válidas: mediana de **37,6 ms**, percentil 95 de **49,3 ms**, máximo de **124,2 ms**, abaixo do limite registrado de 250 ms.
- `StaleMeasurement` e `FutureTimestamp` permaneceram falsos.

## Posição típica e confiança aplicada

Para reduzir a influência das transições, as estatísticas abaixo descartam o primeiro segundo de cada trecho contínuo selecionado. As durações são diferentes; o trecho com uma tag é uma amostra preliminar.

| Condição | Intervalo analisado | Mediana X | Mediana Y | Diferença da mediana inicial | `XYStdDevMeters` |
| --- | --- | --- | --- | --- | --- |
| Duas tags, antes | 31,906–54,253 s | 14,2107 m | 4,7484 m | Referência | 0,25 m |
| Uma tag | 63,373–69,333 s | 14,1322 m | 4,7146 m | 8,55 cm | 0,60 m |
| Duas tags, depois | 70,373–96,735 s | 14,2093 m | 4,7480 m | 0,15 cm | 0,25 m |

A pose estimada do robô acompanhou essa mudança: suas medianas passaram de aproximadamente (14,2105; 4,7484) m para (14,1324; 4,7147) m e retornaram a (14,2093; 4,7479) m.

O código atual atribui 0,25 m a duas ou mais tags até 2 m e 0,60 m a uma tag até 2 m. Os valores registrados nos trechos estáveis correspondem a essa regra. A distância média informada às tags ficou próxima de 1,73–1,74 m.

`XYStdDevMeters` é a incerteza configurada para ponderar a medição; não é o erro real medido no teste. Aumentá-la reduz a confiança na visão durante a fusão, conforme a [documentação da WPILib](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html#tuning-pose-estimators). Neste registro, mesmo recebendo menor confiança, a sequência de leituras com uma tag deslocou a pose estimada.

## Oscilação e mudança de referência são coisas diferentes

Depois da transição, a leitura com uma tag variou pouco em torno da sua própria mediana. O maior ponto de atenção é a mudança da posição típica entre as condições.

Uma comparação adicional usa janelas iguais de aproximadamente seis segundos, iniciadas um segundo depois do começo de cada trecho. A faixa abaixo representa máximo menos mínimo, não erro absoluto:

| Condição | Janela aproximada | Faixa de X da visão | Faixa de Y da visão |
| --- | --- | --- | --- |
| Duas tags, antes | 31,906–37,869 s | 0,82 cm | 0,42 cm |
| Uma tag | 63,373–69,333 s | 1,38 cm | 0,67 cm |
| Duas tags, depois | 70,373–76,335 s | 1,21 cm | 0,53 cm |

O teste mostra uma diferença de posição associada à troca de tags nesse registro. Não identifica sozinho a causa, nem permite escolher qual leitura é mais próxima da posição física correta.

## Próximo teste

**Situação em 14/10/2026:** teste físico adiado a pedido do usuário, que está sem o robô. A comparação individual de cada tag permanece pendente. O trabalho pode continuar em simulação; resultados virtuais não encerram esta validação física.

Manter o robô na mesma posição, sem tocar na câmera nem alterar as configurações:

1. Registrar 30 s vendo as duas tags.
2. Cobrir completamente uma tag no alvo e registrar 30 s vendo somente a outra; anotar o ID que permaneceu visível.
3. Trocar a cobertura e registrar 30 s vendo somente a segunda tag; anotar seu ID.
4. Descobrir ambas e registrar mais 10 s.

Conferir `TagCount = 1` durante cada etapa individual. Cobrir a tag no alvo preserva a visão da outra; cobrir parte da lente ou deixar uma tag parcialmente tampada introduz outra condição no teste.

Essa comparação permite verificar se a diferença se repete com uma tag específica ou com ambas individualmente. Uma posição física medida será necessária para decidir qual estimativa está mais correta. Manter os filtros atuais durante essa coleta; o registro não justifica bloquear todas as leituras com uma tag.

## Limitações

- O trecho contínuo com uma tag foi de sete segundos, abaixo dos 30 s propostos.
- Os 4,70 s sem tags não foram incluídos nas estatísticas de posição válida.
- Algumas colunas de timestamps e diagnósticos estão inteiramente vazias. Ausência foi tratada como indisponibilidade, nunca como zero.
- As linhas do CSV podem repetir a mesma leitura. O número aceito foi calculado pelo contador, não pelo número de linhas `ACCEPTED`.
- Pose e contagem de tags descrevem a leitura atual; os resultados de fusão e a incerteza aplicada podem se referir a uma medição anterior na fila. Por isso, foram comparados trechos estáveis.
- Esta análise usa apenas a câmera traseira e não valida a fusão de duas câmeras.
- Nenhum código do robô foi alterado nesta análise.
