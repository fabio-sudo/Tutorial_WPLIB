# Medição do curso do pivô CAN 32 — 29/10/2026

> Histórico. A medição posterior de 132° e a implementação de ±130° estão em [PIVO_130_GRAUS_E_LIBERACAO_DO_TIRO_2026-10-29.md](PIVO_130_GRAUS_E_LIBERACAO_DO_TIRO_2026-10-29.md).

## Registro dos extremos

Arquivo: `C:/Users/Admin/Desktop/log chooter rotor/tuner-plot.csv`.
SHA256: `779462a27c66ea7fc009fba5924e1948e94f8eac66ad1acf5a6c2d8785f1da44`.
8.842 linhas; tempo 215,228–319,972 s. A equipe confirmou que o teste movimentou o shooter completo e sua fiação.

| Grandeza | Voltas do rotor |
|---|---:|
| Primeira leitura de RotorPosition | -0,001465 |
| Maior posição positiva | +9,165527 |
| Menor posição negativa | -10,637207 |
| Curso entre os extremos | 19,802734 |
| Deslocamento positivo a partir da primeira leitura | +9,166992 |
| Deslocamento negativo a partir da primeira leitura | -10,635742 |
| Última posição | -0,785156 |

Os extremos de Position e RotorPosition coincidem. Não se atribui direita/esquerda ao sinal sem confirmar o sentido físico. Não foi detectado um novo zero a partir destes valores; os limites devem usar a mesma referência física de zero do ensaio.

Proposta inicial de margem: 0,5 volta do rotor para dentro de cada extremo. Resulta em **mínimo -10,137207 / máximo +8,665527**, em coordenadas do encoder do ensaio. São limites propostos, não limites certificados para velocidade/carga e não foram aplicados ao controle nesta análise.

## Conversão angular ainda precisa ser conciliada

A redução informada anteriormente como total foi 9:1. Nessa hipótese, cada volta do rotor equivale a 40° da mira. Os extremos observados corresponderiam a +366,62108° e -425,48828°. Isso não é compatível com a hipótese anterior de aproximadamente ±90° de curso físico. O CSV mede o rotor e não mede diretamente o ângulo do shooter.

Antes de ampliar o curso automático, confirmar quanto a mira realmente girou e, se necessário, a transmissão completa até a parte giratória. Não calcular uma redução final a partir de um ângulo apenas estimado. A discrepância afeta tanto a mira como a simulação e o perfil de movimento.

## Registro para vencer atrito

Arquivo: `C:/Users/Admin/Desktop/log duty cycle vencendo atrito/tuner-plot.csv`.
SHA256: `8ba19f46a22e944de96dd6ab08b8128730eb0af6f34c4d8fb0a369dfbb21cb19`.
22.911 linhas; tempo 353,827–480,878 s; CAN 32.

Houve movimento com DutyCycle perto de +0,03 e -0,03. Também houve trechos sem mudança de posição nesses níveis; não é uma compensação universal de atrito. O registro inteiro contém pedidos medidos de aproximadamente -0,041016 a +0,040039, tensão de -0,51 a +0,50 V e corrente de estator de -3,18 a +5,98 A.

0,03 equivale a 3% da saída, não a 0,03 V. A 12 V seriam nominalmente 0,36 V, mas caracterização deve usar a tensão efetivamente medida. O ensaio com motor e redutor juntos não justifica multiplicar esse percentual pela redução nem aplicá-lo automaticamente como kG ou kS.

## Estado desta análise

Atualização da equipe: aproximadamente **128° do zero até o limite de um lado**, e não o curso entre os dois extremos. Ainda falta associar essa medição ao extremo positivo ou negativo do rotor. Se for +9,165527 voltas, a redução total estimada é 25,778:1; se for -10,637207 voltas, é 29,917:1. Esses números são estimativas de calibração, não relações de engrenagens confirmadas. Não aplicar 128° aos dois lados sem medir o segundo extremo.

Nenhuma ampliação dos limites, alteração de redução ou deploy foi feita com base nestes dois arquivos. O controle permanece com a configuração anterior enquanto se esclarece o ângulo real do ensaio.
