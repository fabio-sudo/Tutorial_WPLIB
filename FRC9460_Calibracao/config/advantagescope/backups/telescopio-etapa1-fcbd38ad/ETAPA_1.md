# Telescópio — etapa 1: montagem articulada no AdvantageScope

## O que foi preparado

O modelo do telescópio foi dividido sem alterar as malhas ou os materiais:

| Arquivo | Conteúdo |
| --- | --- |
| `model.glb` | N1: conjunto externo e acionamentos |
| `model_0.glb` | N2: estágio intermediário |
| `model_1.glb` | N3: estágio interno |
| `config.json` | Orientação do modelo e referências das duas partes móveis |

O asset foi instalado em `config/advantagescope/assets/Robot_Telescopio` no projeto de calibração. No AdvantageScope ele se chama **FRC9460 - Telescopio - Etapa 1**.

O GLB original foi preservado em `config/advantagescope/modelo_original/Mechanism/Telescopio.glb`. A antiga pasta `assets/Mechanism` foi retirada do catálogo porque seu nome/formato provocava o aviso de falha ao carregar assets. O hash SHA-256 do GLB permaneceu igual.

## Como olhar esta etapa

1. No AdvantageScope, use **File > Import Layout...** e selecione `Telescopio-Etapa1-Layout.json`, nesta pasta. Esse layout contém uma vista 3D de bancada e um gráfico da extensão visual.
2. Use **File > Open Log(s)...** e selecione `Telescopio-Etapa1-Demo.wpilog`.
3. Na aba **TELESCOPIO - Etapa 1 visual**, use a reprodução ou clique em tempos diferentes na linha do tempo. O N1 permanece como base; N2 e N3 mudam de posição.
4. Compare 0 s, 2 s e 4 s: a demonstração faz um movimento parcial e volta a se repetir. Verifique se as peças continuam encaixadas.

A aba 3D Field exibe as malhas GLB. A aba Mechanism utiliza um desenho Mechanism2d e poderá ser acrescentada na etapa do código Java.

Para voltar às visualizações do swerve e da visão, importe `config/advantagescope/FRC9460-Simulacao3D.json`. Esse arquivo e o modelo completo do robô foram preservados.

## O que a demonstração significa

O log tem 20 segundos e publica uma pose fixa para a base e um array de duas Pose3d para os componentes. O AdvantageScope aplica cada pose à parte correspondente.

O roteiro demonstra um movimento parcial de 250 mm no N2 e 500 mm absolutos no N3, com ambos se movendo ao mesmo tempo. Esses números e essa relação são **somente um teste de visualização**, não o curso completo nem a cinemática validada do mecanismo real. A origem vertical do conjunto de bancada foi colocada na parte inferior do N1; não representa a posição de instalação no robô completo.

Esta etapa não calcula tensão, torque, corrente, gravidade, aceleração ou resposta de um controlador. A movimentação segue uma curva previamente definida para conferir os componentes.

## Dados informados para a próxima etapa

- Motor: **1 Kraken X44**.
- Redução do motor: **5:1**.
- Diâmetro do tambor: **35 mm** (raio de 17,5 mm).
- Massa inicial a considerar: **5 kg**, informada pelo usuário.
- Curso informado: **624 mm**; falta esclarecer se é total da ponta ou por estágio.
- Falta esclarecer a transmissão do movimento entre N2 e N3 (juntos, sequenciais, ou outra relação).

O log registra esses dados para referência. Eles ainda não comandam a física desta demonstração.

## Conferências realizadas

- Os três GLBs foram importados novamente no Blender.
- Foram preservados os 1.221.048 triângulos instanciados da montagem.
- As transformações dos componentes recompõem a posição original do CAD.
- Altura aproximada na posição exportada: **2,1145 m**. Essa é uma dimensão da montagem, não o curso.
- A parte inferior do modelo de bancada coincide com Z = 0.
- O modelo aparece cadastrado no seletor de robôs do AdvantageScope.
- O original foi preservado com o mesmo SHA-256.

A fonte de preparação e a prévia renderizada ficam também em `C:/Users/Admin/Documents/ChatGPT/FRC/telescopio_etapa1`.

Referência do formato de componentes: https://docs.advantagescope.org/more-features/custom-assets/#articulated-components
