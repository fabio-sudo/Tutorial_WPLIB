# Pose estimada e Limelight

Criado em 30/10/2026, depois de dois testes (28 e 29/10) em que a câmera via as
tags e a pose não era corrigida, ou era corrigida **do lado errado** (o hub
aparecia a 9 ou 10 m).

## Arquivos

| Aplicativo | Arquivo | Uso |
|---|---|---|
| AdvantageScope | `config/advantagescope/layouts/FRC9460-PoseLimelight.json` | 9 abas: campo 2D e 3D com os fantasmas, câmera, correção, por que rejeita, inicialização, heading, erro |
| Elastic | `config/elastic/layouts/FRC9460-PoseLimelight.json` | 2 telas ao vivo: pose e correção, heading e inicialização |

## Como a pose é montada

1. **Odometria + Pigeon** andam o estimador (`Drive`, 50 Hz).
2. **MegaTag2** (`VisionIOLimelight`): a Limelight resolve X e Y recebendo o heading do Pigeon. O `Vision` valida cada medição e o `Drive` aplica só X e Y.
3. **MegaTag1**: a Limelight resolve X, Y e **heading pelas próprias tags**. Antes de 30/10 o código não usava isso, e é a única fonte que pode desmentir o Pigeon.

O MegaTag2 aceita o heading do Pigeon como verdade. Com o heading errado a pose
sai **espelhada em volta das tags** e ainda assim é "corrigida", e o shooter
confia nela. Por isso o heading precisa ser conferido pelo MegaTag1.

## Inicialização automática (nova)

Regras, todas em `VisionConstants` (região INICIALIZAÇÃO AUTOMÁTICA):

- só com o robô **desabilitado e parado** (giroscópio bom);
- leitura MegaTag1 com **2 ou mais tags**, a até 4,5 m, dentro do campo;
- **6 quadros novos seguidos** concordando (posição até 15 cm, heading até 3°);
- só aplica se o estimador estiver desalinhado: heading além de 6° ou posição além de 0,75 m. Alinhado, não faz nada, então não há reset repetido nem salto;
- aplica X, Y **e heading** de uma vez; o histórico de visão é descartado e a referência do piloto volta ao campo.

**Durante o jogo o heading nunca muda** (giraria a referência do piloto). Se o
Pigeon divergir das tags por mais de 1 s com o robô parado, vira o alerta
"Visão: o heading do robô difere das tags…" no Elastic. O reset manual
(Start + A) continua valendo.

Depois da inicialização, `Pose confiável` só acende no primeiro MegaTag2
aplicado com o heading novo: o shooter não atira com a pose recém-redefinida.

## Procedimento de bancada

Robô ligado, Limelight conectada, duas tags do hub (25 e 26, ou 9 e 10) à vista a menos de 4,5 m.

1. **Elastic, tela "01 Pose e correção".** `Camera conectada` e `Vendo tags` acesos; `Tags` ≥ 2.
2. **Desabilitado e parado** por uns 0,3 s. Na tela "02 Heading e inicialização", `Inicialização pelas tags` passa por `JUNTANDO_FRAMES` e termina em `APLICADA` (uma vez, `Aplicacoes` = 1) ou `ALINHADA` (já estava certa).
3. **Confira o campo** (aba P1): a pose estimada deve cair onde o robô está de fato, olhando para onde ele olha.
4. **Habilite.** `Correção recente` acende em menos de 1 s, depois `Pose confiável`.
5. **Mova devagar** e confirme que a pose acompanha (P1) e que `Erro pose × visão` (P8) fica pequeno.

Se algo não acende, leia `Vision/Diagnostico` (aparece no topo da tela 01 e como aviso na Driver Station):

| Código | Causa provável | Ação |
|---|---|---|
| `SEM_CONEXAO` | Heartbeat da câmera parado no NetworkTables | Alimentação, cabo, endereço da Limelight |
| `SEM_TAG_ACEITA` | A câmera não usou nenhuma tag: nenhuma à vista, ou fora de `LimelightConstants.VALID_TAG_IDS` | Veja a imagem da câmera. Ela é fixa na traseira: com o robô muito girado em relação ao hub, as tags dele saem do quadro |
| `VE_TAG_SEM_POSE` | Vê a tag, mas a pose MegaTag2 veio sem tags | Pipeline sem localização 3D; mapa do campo da câmera; tag fora do mapa |
| `GIROSCOPIO_INDISPONIVEL` | Pigeon sem leitura | Sem ele não há odometria nem MegaTag2 |
| `GIRANDO_RAPIDO` | Giro acima do limite | Normal enquanto gira |
| `CAPTURA_ATRASADA` / `TIMESTAMP_NO_FUTURO` | Relógios da Limelight e do robô fora de sincronia | NetworkTables, rede |
| `REJEITADA: <filtro>` | Um filtro recusou a medição | Aba P5: cada faixa acesa é um filtro |
| `VALIDA_MAS_NAO_APLICADA` | A medição passou, mas a fila ou o Drive recusou | `Drive/Vision/LastResult` |
| `OK_CORRIGINDO` | Funcionando | |

## Critérios de aceite (valores iniciais, a confirmar no robô)

- Parado vendo 2 tags: erro de heading pelas tags abaixo de 3° e `Inicialização` em `ALINHADA`.
- Com tags à vista, `Correção recente` acesa na maior parte do tempo.
- `Erro pose × visão` abaixo de 0,3 m parado.

## Limites

- Com **uma tag só** o MegaTag1 é ambíguo e não inicializa. Os campos `Vision/*/MegaTag1/*` e `Vision/AutoInit/*` só existem em logs gravados depois de 30/10/2026.
- Medição pelas tags perto de uma parede lateral, ou longe (mais de 4,5 m), é descartada para inicializar.
- **A causa da falta de correção na sessão das 15:18 de 29/10 não foi confirmada**: não havia log AdvantageKit do robô. O diagnóstico acima nomeia a causa no próximo teste.
- Na simulação o `PhotonVision` multi-tag faz o papel do MegaTag1.
