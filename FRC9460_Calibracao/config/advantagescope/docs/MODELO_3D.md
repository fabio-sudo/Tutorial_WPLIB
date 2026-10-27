# Modelo do robô no AdvantageScope

O asset `mecanismos/Robot_Robo_9460` contém o modelo **FRC9460 - Robo 2026**.
O GLB atual vem da nova montagem exportada pelo Unity, com a pose de mundo
da raiz removida. O arquivo recebido e a montagem anterior foram preservados
em `build/model-diagnostics/backup-20261015-112443`.
Todas as malhas e materiais estão nos próprios GLBs; não há arquivos externos.

O asset está dividido em uma base fixa e dois mecanismos articulados (intake e
torre do shooter); veja [Mecanismos articulados](#mecanismos-articulados).
O modelo inteiro, de onde a divisão é gerada, fica em
`modelos_originais/Robo_9460`, fora de `mecanismos/` para o
AdvantageScope não carregar o robô duas vezes. Os caminhos deste guia são
relativos a `config/advantagescope` (veja o [LEIA-ME](../LEIA-ME.md)).

## Origem e orientação

- A escala original está em metros e foi preservada.
- Rotação de +90° em X: converte o eixo vertical Y do GLB para Z no campo.
- O centro horizontal usa a média das posições dos quatro módulos swerve.
- O deslocamento vertical coloca o ponto mais baixo do modelo sobre o piso.
- Dimensões aproximadas da montagem atual: 1,043 m em X, 0,838 m em Y e
  0,695 m em Z, na posição em que seus mecanismos foram exportados. A altura
  antiga de 1,154 m incluía o cubo de VR `IntakeGrabHandle`, agora removido.
- A frente foi interpretada como +X do CAD, onde o intake se projeta. Confira
  visualmente contra a frente física antes de usar essa orientação como referência.

Esses ajustes ficam em `config.json`. O deslocamento é aplicado depois da rotação.
Não use o centro da caixa de todo o modelo como centro do Drive: o intake avança
além do chassi. As dimensões acima incluem essa projeção.

## Instalação e uso

Neste computador, o AdvantageScope está configurado para ler **a pasta
`config/advantagescope/mecanismos` do projeto**. Esse caminho personalizado substitui
a pasta padrão `userAssets`: trocar somente o GLB na pasta padrão não atualiza
o modelo exibido. Edite a cópia da pasta ativa.

Para localizar a pasta ativa em outro computador, use
**App > Show Assets Folder** e copie a pasta inteira `Robot_Robo_9460`.
Também é possível apontar **App > Use Custom Assets Folder** para a pasta
`config/advantagescope/mecanismos` do projeto, conforme a documentação oficial.

As abas `SIM 3D - Fusao`, `SIM 3D - Aplicadas` e `Robo 9460 3D` do
layout `layouts/FRC9460-Simulacao3D.json` usam esse modelo. Importe o layout por
**File > Import Layout** depois de cadastrar o asset.

A geometria atual tem cerca de 762 mil triângulos. A simplificação automática do
AdvantageScope permanece habilitada para reduzir o custo da visualização.

## Exportação pelo Unity

A exportação recebida continha a raiz `9460` na posição Unity
`[6.754, 0.22, -2.762]`, girada 180° em Y. Esse é o posicionamento do robô
na cena do editor. O AdvantageScope já posiciona o modelo pela pose registrada
pelo Drive; acumular as duas transformações desenha o robô deslocado/invertido.

`scripts/preparar_modelo_unity.py` remove somente a translação e a rotação dessa raiz,
preservando os filhos, a montagem dos mecanismos, as malhas e os materiais.
Os bytes dos chunks binários permanecem idênticos. A rotação e o deslocamento
de `config.json` continuam convertendo a montagem para o referencial do robô.

Para uma próxima exportação compatível, deixe a raiz em posição zero, rotação
identidade e escala 1 no Unity, mantendo as posições locais dos componentes.
Outra opção é executar o preparador com `--source` e `--output`, usando arquivos
diferentes. Confira sempre a montagem antes de substituir o asset ativo.

Depois de preparar uma nova exportação, grave o GLB inteiro em
`modelos_originais/Robo_9460/model.glb` e execute
`scripts/preparar_componentes.py` para gerar de novo a base e os mecanismos.

Após atualizar o GLB, use **View > Reload** no AdvantageScope para descarregar
a geometria anterior da memória e carregar a nova montagem.

## Mecanismos articulados

O AdvantageScope não lê animações do GLB. Cada peça móvel é um arquivo
separado (`model_N.glb`) e o código publica uma pose por peça, relativa ao robô.

| Arquivo | Conteúdo | Movimento |
|---|---|---|
| `model.glb` | chassi, swerve, centrífuga, base do shooter, Kraken X44 + redução + mancais do pivô do intake | fixo |
| `model_0.glb` | intake: braços, rolo, proteções e motor do rolo | gira no pivô (sobe e desce) |
| `model_1.glb` | torre do shooter: tudo acima da engrenagem grande `Spur Gear2` | gira no eixo vertical |

`preparar_componentes.py` separa os nós pelo nome do CAD, mede os pivôs e
escreve `components` no `config.json`. As malhas não são alteradas. O cubo
verde translúcido `IntakeGrabHandle` (alça de VR do Unity) é removido.

Pivôs no referencial do robô (X frente, Y esquerda, Z para cima; origem no
centro do chassi, no piso), medidos automaticamente:

| Componente | Pivô (m) | Eixo | Ângulo 0 | Sentido positivo |
|---|---|---|---|---|
| Intake (`model_0`) | (0,2785; 0; 0,2389) | Y | abaixado, como no CAD | `Rotation3d(0, -subida, 0)` levanta |
| Torre (`model_1`) | (−0,1439; 0,1449; 0,3475) | Z | posição do CAD | anti-horário visto de cima |

- Intake: ponto médio dos dois rolamentos coaxiais aos Kraken X44 do pivô.
- Torre: centro da engrenagem `Spur Gear2`, na face inferior dela. A fiação
  limita o giro a **±90°** a partir do ângulo 0; o código deve respeitar esse
  limite (o AdvantageScope desenha qualquer ângulo recebido).

Cada componente usa `zeroedRotations` igual a `rotations` e
`zeroedPosition = position - pivô`, então o pivô fica na origem e a pose
publicada é simplesmente (posição do pivô, rotação da junta). Sem dados de
componentes, o AdvantageScope desenha as peças na posição do CAD: o robô
continua aparecendo inteiro.

### Ver os mecanismos sem código

```bash
python config/advantagescope/scripts/gerar_log_demo_componentes.py
```

O script grava um log por objeto, ao lado dos GLBs. O do robô é
`mecanismos/Robot_Robo_9460/Robo_9460-Demo.wpilog` (24 s): o intake sobe até
90° e desce a cada 8 s, e a torre varre ±90° (limite da fiação). Importe o
layout `layouts/FRC9460-Simulacao3D.json`, abra o log por
**File > Open Log(s)** e use as abas **Robo 9460** (intake visto de lado, em
2D) e **Robo 9460 3D** (GLBs). As abas `SIM 3D - Fusao` e
`SIM 3D - Aplicadas` também têm os componentes do robô.

### Quando o código Java publicar

O `ShooterVisualizer` já publica esta chave (veja [SHOOTER.md](SHOOTER.md)).
No CAD, a marca `FrenteShooter` aponta para 175,5°, não 180°: o código desconta
essa diferença para o desenho mostrar o mesmo ângulo usado na mira. O intake
fica na posição do CAD até ter subsistema.

O layout espera um `Pose3d[]` em `Mecanismos/Componentes`, na ordem dos
arquivos (`model_0`, `model_1`):

```java
Logger.recordOutput("Mecanismos/Componentes", new Pose3d[] {
    new Pose3d(0.2785, 0.0, 0.2389, new Rotation3d(0.0, -intakeSubidaRad, 0.0)),
    new Pose3d(-0.1439, 0.1449, 0.3475, new Rotation3d(0.0, 0.0, torreRad))
});
```

## Telescópio da equipe 9458

`scripts/preparar_telescopio_9458.py` divide
`modelos_originais/Telescopio_9458/Telescopio.glb` (sem alterá-lo) no asset
`mecanismos/Robot_Telescopio_9458`, chamado **FRC9458 - Telescopio** no
AdvantageScope:

| Arquivo | Parte |
|---|---|
| `model.glb` | tubo 4" (externo) e acionamento, fixo |
| `model_0.glb` | tubo 3" (meio) |
| `model_1.glb` | tubo 2" (interno) |

O CAD foi exportado recolhido: extensão 0 é o telescópio fechado. O motor e a
redução ficam na base, e os tubos saem para cima pela ponta oposta; no GLB
original o motor está em +Y, por isso o `config.json` gira −90° em X. É um
asset separado, com o ponto mais baixo no piso; não está montado no robô. A base
fica em `Telescopio9458/Pose` (na origem, no log de demonstração, até haver
uma posição de montagem). O movimento é só translação, então
a pose de cada tubo é `(0, 0, altura)` e o tubo 2" soma a altura do tubo 3".

A montagem é em cascata (os dois estágios sobem juntos, 10 cm + 10 cm), com
Kraken X60, redução 16:1 e coroa 16T de corrente #25: a corrente anda
16 × 6,35 = 101,6 mm por volta da coroa, e cada volta do motor move cada
estágio 101,6 ÷ 16 = 6,35 mm. O fim de curso fica em 78,4 voltas do rotor:
0,498 m por estágio e 0,996 m de deslocamento da ponta (altura aberta
0,780 + 0,996 ≈ 1,776 m). A partir do encoder:

```java
double estagio = rotacoesMotor * TelescopioConstants_9458.STAGE_METERS_PER_MOTOR_ROTATION; // metros
Logger.recordOutput("Telescopio9458/Pose", poseDaBaseNoCampo);
Logger.recordOutput("Telescopio9458/Componentes", new Pose3d[] {
    new Pose3d(0, 0, estagio, new Rotation3d()),     // tubo 3"
    new Pose3d(0, 0, 2 * estagio, new Rotation3d())  // tubo 2" (cascata)
});
```

O log de demonstração, `mecanismos/Robot_Telescopio_9458/Telescopio_9458-Demo.wpilog`,
para em 10 cm + 10 cm, sobe até o fim de curso (78,4 voltas) e fecha. Use as abas **Telescopio 9458**
(desenho 2D) e **Telescopio 9458 3D** (GLBs); parâmetros e detalhes em
[telescopio_9458/ADVANTAGESCOPE.md](telescopio_9458/ADVANTAGESCOPE.md).

### Grupos por estágio para o Blender

`python config/advantagescope/scripts/preparar_telescopio_9458.py --agrupado`
grava também `modelos_originais/Telescopio_9458/Telescopio_9458_por_estagio.glb`:
o modelo inteiro,
nas posições originais, com um grupo por estágio (a mesma divisão do asset):

| Grupo | Peças | Vira |
|---|---|---|
| `Estagio1_Fixo_Tubo4` | tubo 4", Kraken X60 + redução 2x, encoder REV, placas, coroas e correntes | `model.glb` |
| `Estagio1_REVISAR_presilhas_corrente` (dentro do estágio 1) | tensores Spartan, bloco `61-01-04` e seus parafusos e porcas | revisar |
| `Estagio2_Tubo3` | tubo 3" e suas peças | `model_0.glb` |
| `Estagio3_Tubo2` | tubo 2" e suas peças | `model_1.glb` |

As presilhas provavelmente prendem a corrente ao estágio 2 pelo rasgo do tubo
4". Elas ficam no estágio 1 porque a corrente, que num modelo rígido não pode
se deformar, também fica parada lá; no estágio 2, elas subiriam e se
separariam da corrente. Se preferir, arraste o grupo para o estágio 2.

No Blender, para cada estágio: **File > Import > glTF 2.0**, clique com o
botão direito no grupo do Outliner > **Select Hierarchy**, depois
**File > Export > glTF 2.0** com *Format: glTF Binary* e
*Include > Limit to: Selected Objects*. Não mova as peças: a posição
exportada é o zero de cada estágio.

## Relação com a simulação

Os eixos das câmeras continuam vindo das poses publicadas pelo Java, sem
duplicar suas medidas neste asset.

O desenho não muda as posições das tags, a confiança da visão, o campo de visão
PhotonLib, a odometria ou a física. Também não cria automaticamente oclusões ou
colisões. Essas características pertencem ao simulador e ao código do robô.

Referência: [Custom Assets do AdvantageScope](https://docs.advantagescope.org/more-features/custom-assets/).
