# Elastic — FRC9460

Painéis do Elastic para usar **junto** com os layouts do AdvantageScope:
o Elastic mostra o agora (números, verde/vermelho, alertas) durante o treino e
no pit; o AdvantageScope mostra o histórico e as curvas sobrepostas.

| Elastic | Use junto com (AdvantageScope) | Abas |
|---|---|---|
| `FRC9460-Analise` | `FRC9460-Campo2D3D.json` | **Painel único de análise**: 01 Pré-jogo, 02 Campo e câmera, 03 Tiro, 04 Mecanismos, 05 Saúde do robô |
| `FRC9460-ValidacaoRobo` | `FRC9460-ValidacaoRobo.json` | Operação, Visão, Movimento, Mira e rodas, Último tiro, Gravação e motores, Chegada 2 m |
| `FRC9460-ConsistenciaDeTiro` | `FRC9460-ConsistenciaDeTiro.json` | Constância dos tiros: velocidade, mira e acerto |
| `FRC9460-PoseLimelight` | `FRC9460-PoseLimelight.json` | Câmera, correção da pose e heading |
| `FRC9460-Shooter` | `FRC9460-Shooter.json` | Shooter, Visão, Real x Modelo, Diagnóstico |
| `FRC9460-CicloDaBola` | `FRC9460-CicloDaBola.json` | Ciclo da bola, Tiro e visão, Real x Modelo, Diagnóstico |
| `Telescopio9460` | `Telescopio9460-Simulacao.json` / `-TesteControle.json` | Telescópio, Real x Modelo, Diagnóstico |
| `FRC9460-Diagnostico` | qualquer um | todos os motores numa tela (pit) |

## Abrir

**Direto do robô ou da simulação** (o `Robot.java` serve os painéis na porta 5800).
O deploy copia `config/elastic/layouts/` para o robô: mudou um painel, faça o deploy.

1. Robô ligado (ou simulação rodando). No Elastic, em *Settings*, escolha o
   endereço do robô (time 9460) ou `localhost` para a simulação.
2. **File > Download Layout** e escolha o painel da lista.

**Pelo arquivo**: **File > Open Layout** > `config/elastic/layouts/<painel>.json`.

## O que olhar

- **Painel de análise**: comece pela tela **01 Pré-jogo**, que responde se dá para
  ligar: câmera, giroscópio, zeros, gravação do log, bateria, CAN e o resultado de
  cada motor. A **02 Campo e câmera** tem o mapa (robô, MegaTag2 e hub) e o vídeo da
  Limelight, que só existe no robô real. A **05 Saúde do robô** mostra o tempo de
  ciclo: acima de 20 ms são os loop overruns.
- **Visão**: `Corrigiu desde o reset` e `Correção recente (1 s)` verdes antes de
  atirar. "Idade da última correção" subindo = a Limelight parou de corrigir.
- **Tiro e visão**: em cada tiro, se havia visão recente e (na simulação) o
  erro da pose. Muitos "Tiros SEM visão recente" explicam erros de mira.
- **Real x Modelo**: gráfico da diferença entre o mecanismo e a física com a
  mesma tensão. `Divergindo` vermelho por mais de 0,5 s = olhar o mecanismo.
  O significado de cada diferença está em
  [ESTRUTURA_DO_CODIGO.md](../../ESTRUTURA_DO_CODIGO.md#4-modelo-em-paralelo-real-x-simulado).
- **Diagnóstico**: `MotorRespondendo` vermelho = motor sumiu do CAN; o widget de
  alertas mostra qual. O telescópio 9460 desligado em `CanIds` aparece como aviso.

## Mudar um painel

Os painéis `Analise`, `ValidacaoRobo`, `ConsistenciaDeTiro` e `PoseLimelight` são
gerados junto do AdvantageScope por
`python config/advantagescope/scripts/preparar_layout_validacao_robo.py`.
Veja o [guia de validação](../advantagescope/docs/VALIDACAO_ROBO.md).

Para os outros, não edite o JSON à mão: mude `preparar_layouts_elastic.py` e rode, na raiz:

```bash
python config/elastic/preparar_layouts_elastic.py
```

Depois rode o `cicloDaBolaTest`: ele confere se cada tópico existe no log com o
tipo certo (`double`, `int`, `boolean`, `string`, `double[]`, `int[]`).

Nomes dos tópicos: o AdvantageKit publica as saídas em
`/AdvantageKit/RealOutputs/<chave>` (no log, `/RealOutputs/<chave>`). Os alertas
da WPILib ficam em `/SmartDashboard/Alerts`. O mapa lê `Campo/Robot`, e cada outro
filho de `Campo/` vira um objeto (`[x, y, graus]`); vazio esconde o objeto.
