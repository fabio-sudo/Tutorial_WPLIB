# Telescópio — etapa 1: montagem articulada no AdvantageScope

> Este documento descreve a demonstração visual original. A simulação Java atual,
> com movimento simultâneo, controle e física aproximada, está em
> [SIMULACAO_JAVA.md](SIMULACAO_JAVA.md).

## O que foi preparado

O modelo do telescópio foi dividido sem alterar as malhas ou os materiais:

| Arquivo | Conteúdo |
| --- | --- |
| `model.glb` | N1: conjunto externo e acionamentos |
| `model_0.glb` | N2: estágio intermediário |
| `model_1.glb` | N3: estágio interno |
| `config.json` | Orientação do modelo e referências das duas partes móveis |

O asset foi instalado em `config/advantagescope/mecanismos/Robot_Telescopio_9460` no projeto de calibração. No AdvantageScope ele se chama **FRC9460 - Telescopio - Etapa 1**.

O GLB original foi preservado em `config/advantagescope/modelos_originais/Telescopio_9460/Telescopio.glb`. A antiga pasta `assets/Mechanism` foi retirada do catálogo porque seu nome/formato provocava o aviso de falha ao carregar assets. O hash SHA-256 do GLB permaneceu igual.

## Como olhar esta etapa

1. No AdvantageScope, use **File > Import Layout...** e selecione `config/advantagescope/layouts/Telescopio-Etapa1-Layout.json`. A primeira aba é **Mechanism - Telescopio**; o layout inclui também uma vista 3D de bancada e um gráfico dos cursos.
2. Use **File > Open Log(s)...** e selecione `config/advantagescope/mecanismos/Robot_Telescopio_9460/Telescopio_9460-Demo.wpilog`.
   As abas **Telescopio 9460** (2D) e **Telescopio 9460 3D** do layout principal
   `layouts/FRC9460-Simulacao3D.json` também mostram este log.
3. Na aba **Mechanism - Telescopio**, use a reprodução ou clique em tempos diferentes na linha do tempo. **Azul = N1**, **laranja = N2**, **verde = N3**. O desenho simplificado mostra a extensão dos estágios. Na aba **TELESCOPIO - Etapa 1 visual**, o mesmo movimento aparece no GLB 3D.
4. Compare **0 s** (recolhido), **6 s** (N2 estendido e N3 recolhido) e **10 s** (ambos estendidos). A descida recolhe N3 e depois N2. Verifique se as peças continuam encaixadas.

A aba **Mechanism** já está configurada com o campo `/RealOutputs/Telescopio/Mecanismo2d`, no formato Mechanism2d. O log publica dimensões, fundo, raiz e os três segmentos articulados. As extensões usam os mesmos valores da animação 3D. O desenho representa comprimentos aparentes dos estágios; a sobreposição dos tubos é mostrada no GLB da aba 3D Field.

Para voltar às visualizações do swerve e da visão, importe `config/advantagescope/layouts/FRC9460-Simulacao3D.json`. Esse arquivo e o modelo completo do robô foram preservados.

## O que a demonstração significa

O log tem 20 segundos e publica uma pose fixa para a base e um array de duas Pose3d para os componentes. O AdvantageScope aplica cada pose à parte correspondente.

O usuário informou **624 mm no N2 e 628 mm no N3**, com movimento sequencial. A demonstração representa N2 subindo primeiro e N3 depois; na descida, N3 recolhe primeiro. A ordem de qual estágio começa foi adotada para esta visualização e pode ser corrigida caso o mecanismo real comece pelo N3.

O curso de N3 é medido em relação a N2. Quando N2 sobe, carrega N3 junto; por isso o deslocamento total da ponta pode chegar a **624 + 628 = 1.252 mm**. O gráfico mostra a extensão de N2, a extensão relativa de N3 e a soma. Todos os campos de distância do log estão em metros.

Os tempos da animação são demonstrativos. A origem vertical do conjunto de bancada foi colocada na parte inferior do N1; não representa a posição de instalação no robô completo.

Esta etapa não calcula tensão, torque, corrente, gravidade, aceleração ou resposta de um controlador. A movimentação segue uma curva previamente definida para conferir os componentes.

## Dados informados para a próxima etapa

- Motor: **1 Kraken X44**.
- Redução do motor: **5:1**.
- Diâmetro do tambor: **35 mm** (raio de 17,5 mm).
- Massa inicial a considerar: **5 kg**, informada pelo usuário.
- Curso do N2 em relação ao N1: **624 mm**.
- Curso do N3 em relação ao N2: **628 mm**.
- Deslocamento máximo da ponta em relação à posição recolhida: **1.252 mm**.
- Movimento sequencial, conforme informado pelo usuário. A demonstração adota N2 antes de N3 na subida.

Para a simulação física, a ligação do cabo e suas polias ainda determinará a relação entre o giro do tambor e o deslocamento de cada estágio. Os cursos e a sequência, sozinhos, não estabelecem essa vantagem mecânica.

O log registra esses dados para referência. Eles ainda não comandam a física desta demonstração.

## Conferências realizadas

- Os três GLBs foram importados novamente no Blender.
- Foram preservados os 1.221.048 triângulos instanciados da montagem.
- As transformações dos componentes recompõem a posição original do CAD.
- Altura aproximada na posição exportada: **2,1145 m**. Essa é uma dimensão da montagem, não o curso.
- A parte inferior do modelo de bancada coincide com Z = 0.
- O modelo aparece cadastrado no seletor de robôs do AdvantageScope.
- O original foi preservado com o mesmo SHA-256.
- O formato Mechanism2d foi conferido com a função de leitura do AdvantageScope 26.0.0 instalado: três segmentos em cinco instantes, sem extrapolar as dimensões do desenho. As extensões totais conferidas foram 0, 624 e 1.252 mm.

A fonte de preparação e a prévia renderizada ficam também em `C:/Users/Admin/Documents/ChatGPT/FRC/telescopio_etapa1`.

Referência do formato de componentes: https://docs.advantagescope.org/more-features/custom-assets/#articulated-components
