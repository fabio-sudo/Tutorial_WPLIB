# Constância de tiro: velocidade, mira e acerto

Criado em 30/10/2026. O objetivo não é ver um tiro de cada vez, e sim medir o
**quanto cada disparo se parece com o anterior**, no robô, na simulação e no
replay, e saber o que ajustar.

## Arquivos

| Aplicativo | Arquivo | Uso |
|---|---|---|
| AdvantageScope | `config/advantagescope/layouts/FRC9460-ConsistenciaDeTiro.json` | 11 abas: RPM, queda e cadência, estatística da janela, acerto, causas, **3D de onde chegou × onde devia**, replay |
| Elastic | `config/elastic/layouts/FRC9460-ConsistenciaDeTiro.json` | 2 telas ao vivo: números da janela e anotações/gráficos |

AdvantageScope: **File > Import Layout** e **File > Open Log(s)**. Elastic: **File > Open Layout**.
Os dois são gerados por `python config/advantagescope/scripts/preparar_layout_validacao_robo.py`.

## Como funciona

Cada disparo detectado (queda do RPM do top com a centrífuga girando, ver
`FuelTracker`) entra numa **janela dos últimos 10 tiros**. Para cada medida o
log guarda o último valor, a média, o desvio, o mínimo e o máximo em
`Fuel/Consistencia/<Medida>/{Ultimo,Media,Desvio,Minimo,Maximo}`.

| Medida | O que é |
|---|---|
| `VelocidadeSaidaMps` | Velocidade da bola, pelo RPM medido logo antes da bola (a bola tira energia da roda) |
| `TopRpmNoDisparo`, `TopRpmErro` | RPM do top no disparo e (pedido − medido) |
| `QuedaRpmTop` | Quanto o RPM cai quando a bola passa |
| `ErroMiraGraus` | Direção do alvo atual menos direção do shooter (pelo encoder do pivô) |
| `ErroQuedaMetros` | Onde o modelo diz que a bola cai, medido do centro do hub |
| `DistanciaMetros`, `VelocidadeRoboMps`, `IntervaloSegundos` | Contexto: distância, o robô andando, cadência |
| `TaxaAcertoPlanejado`, `TaxaAcertoSensores` | Fração dos tiros da janela que o modelo diz que acertam |
| `Fuel/Real/TaxaAcerto` | Fração anotada pela equipe como acerto (a verdade no robô real) |

### Viés ou dispersão?

Dois problemas diferentes pedem ajustes diferentes. É a primeira coisa a ler.

| O que se vê | Significa | Onde mexer |
|---|---|---|
| **Média** longe do ideal, **desvio** pequeno | Erro sistemático | Calibração: curva de RPM (`TOP_FORCE_SCALE`), offset do shooter, heading |
| **Desvio** grande, média boa | O robô não repete | Mecânica, alimentação, bateria, pivô que não assenta, pose ruidosa. Mexer na curva não resolve |
| `TopRpmErro/Media` ≠ 0 | A roda não faz o RPM pedido | Ganho/kV do top, bateria, ritmo da centrífuga |
| `ErroMiraGraus/Media` ≠ 0 | Mira com viés | Heading (ver `POSE_E_LIMELIGHT.md`), `SHOOTER_OFFSET_*` |
| `ErroMiraGraus/Desvio` alto | Mira instável | Tempo de assentar do pivô, pose ruidosa |
| `VelocidadeSaidaMps/Desvio` alto | Cada bola sai com velocidade diferente | RPM caindo entre tiros (cadência rápida demais), bateria |

O texto `Fuel/Consistencia/Diagnostico` resume isso numa linha (`CONSTANTE`, ou
`VELOCIDADE_INCONSTANTE`, `TOP_ABAIXO_DO_PEDIDO`, `TOP_ACIMA_DO_PEDIDO`,
`MIRA_INCONSTANTE`, `MIRA_COM_VIES`, `ERRO_DE_QUEDA_ALTO`, separados por `+`).
Antes de 4 tiros na janela ele só diz quantos faltam. **Os limites são valores
iniciais, sem medição no robô** (`FuelConstants`, região CONSTÂNCIA): ajuste
depois de algumas sessões.

## O 3D: onde chegou × onde devia (aba C9)

| Elemento | Cor | O que mostra |
|---|---|---|
| Cone `Devia` + anel | verde + amarelo | O centro e a borda da boca do hub, na altura da boca |
| Trajetória e ponto planejados | laranja | O que o código **pediu** (RPM e ângulo alvo) |
| Trajetória e ponto pelos sensores | ciano | O que **saiu** (RPM medido antes da bola, encoder do pivô) |
| Trajetória e ponto da física | vermelho | Só na **simulação**: a física das bolas |
| Linha do centro até o ponto | mesma cor | O vetor do erro de cada tiro |
| Nuvem de cones | magenta | Os pontos da janela de 10 tiros: o tamanho dela é a constância |

**No robô real nenhum sensor vê a bola.** O ponto "pelos sensores" é uma
previsão do modelo balístico (`BALL_SPEED_EFFICIENCY` e `LAUNCH_ANGLE_DEGREES`
ainda não foram medidos). A verdade é a **anotação da equipe**.

## Anotar cada tiro no robô real

Depois de cada tiro, um clique em `/Tuning/Fuel/AnotarTiroReal/<botão>`
(AdvantageScope em modo de ajuste ou o NetworkTables): `Acertou`, `ErrouCurto`,
`ErrouLongo` ou `ErrouDoLado`. Cada clique fica gravado com o número do tiro
(`Fuel/Real/TiroAnotado`). O **tipo** do erro dá a direção do ajuste:

- mais `ErrouLongo` que `ErrouCurto`: força demais;
- só `ErrouDoLado`: mira (heading, offset), não força;
- misturado: dispersão.

## Rotina de coleta

1. Antes, confirme a pose (`POSE_E_LIMELIGHT.md`): com pose errada, todo número daqui é consequência dela.
2. Fixe a distância (meça com trena) e **não mude constantes** durante a série.
3. 10 tiros por distância, anotando cada um. Varie a distância depois.
4. Baixe o log (`VALIDACAO_ROBO.md`, seção "Replay de um log real") e abra com o layout.

## Replay: o que ele responde e o que não responde

Rode o replay do log (`VALIDACAO_ROBO.md`) e abra o `_replay.wpilog`. A aba C10
compara, tiro a tiro, **o que o robô fez** (`RealOutputs`) com **o que o código
atual teria feito** com os mesmos sensores (`ReplayOutputs`).

Exemplo medido em 30/10/2026, no mesmo log, só mudando `TOP_FORCE_SCALE` de 0,95 para 1,10:

| Campo | Original | Replay com 1,10 |
|---|---|---|
| Erro de queda **planejado** (m) | 0,24 | 1,08 |
| Acerto previsto (planejado) | 100% | 0% |
| `TopRpmErro/Media` (pedido − medido) | −211 | +269 |
| Velocidade de saída (m/s), pelos sensores | 8,154 | 8,154 |
| Erro de queda **pelos sensores** (m) | 0,229 | 0,229 |

O que o replay faz: mostra se uma constante nova **pediria** o RPM e o ângulo
certos para as distâncias e poses daquele teste, sem ligar o robô. O que ele não
faz: os sensores gravados não mudam, então o tiro real continua sendo o que
aconteceu. A prova de que a mudança funciona vem do próximo teste no robô.

## Medidas que faltam no robô

**O top pode estar girando acima do pedido.** `SPARK_KV_VOLTS_PER_RPM` usa a rotação
livre do NEO (5676 RPM), mas o top é Neo Vortex (6784 RPM). O kV fica 19,5% acima do
ideal e, sem atrito, o RPM para em 1,07 × alvo: são os 7% da simulação. O robô real
usa o mesmo kV; com atrito o excesso deve ser menor, mas ainda não foi medido.

Não corrija só o kV: o `TOP_FORCE_SCALE` (0,95) foi calibrado em campo com esse erro
embutido, e os tiros ficariam curtos. Faça nesta ordem:

1. Num log real, com a roda estabilizada antes do tiro, calcule
   `razão = Shooter/Top/RpmEsquerdo ÷ Shooter/Top/AlvoRpm`.
2. Troque o kV para `NOMINAL_VOLTAGE / 6784.0` e multiplique `TOP_FORCE_SCALE` pela
   razão. O tiro continua igual, e o RPM passa a bater com o pedido.

**Offset do shooter.** O código usa 14 cm à direita (`SHOOTER_OFFSET_LEFT_METERS =
-0.14`) e o CAD indica 14,5 cm à esquerda. Meça do centro do robô até o eixo do
shooter: se estiver à esquerda, o sinal muda e a mira lateral erra cerca de 28 cm
até corrigir.

**Velocidade da bola.** `BALL_SPEED_EFFICIENCY` (0,465) e o ângulo de saída nunca
foram medidos. Filme um tiro de lado e meça altura e alcance.

## Limites

- Os campos `Fuel/Consistencia/*`, `Fuel/Tiro/*/Chegada`, `Fuel/Real/Errou*` e `Shooter/Mira/ErroAteOAlvoAtualGraus` só existem em logs gravados depois de 30/10/2026.
- Dez tiros por janela dão uma média e um desvio úteis, não estatística rigorosa.
- A velocidade de saída vem de uma soma ponderada de RPM e de uma eficiência estimada: serve para comparar um tiro com outro, não como medida absoluta da bola.
- Na simulação o top roda cerca de 7% acima do pedido (kV do NEO usado no Vortex): `TOP_ACIMA_DO_PEDIDO` aparece lá e é esperado. No robô real vale o que o log mostrar.
