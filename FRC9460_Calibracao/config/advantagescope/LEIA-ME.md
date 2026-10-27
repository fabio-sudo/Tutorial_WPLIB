# AdvantageScope — FRC9460

Tudo que o AdvantageScope usa neste projeto, separado por tipo de arquivo.

O telescópio 9460 agora tem simulação Java com controle de posição e movimento
simultâneo. Use `layouts/Telescopio9460-Simulacao.json` e siga o
[guia de simulação do telescópio](docs/telescopio_9460_etapa1/SIMULACAO_JAVA.md).
Para testar no controle (segurar RB abre, segurar LB fecha), use
`layouts/Telescopio9460-TesteControle.json`.

O telescópio 9458 também tem simulação Java: segure RT para abrir e LT para
fechar, com `layouts/Telescopio9458-Simulacao.json` e o
[guia do 9458](docs/telescopio_9458/ADVANTAGESCOPE.md).

Para testar as câmeras, reais (Limelight) ou virtuais (PhotonVision), use
`layouts/FRC9460-Cameras.json` e o [guia das câmeras](docs/CAMERAS.md).

O shooter (pivô, top, tower e centrífuga) tem simulação Java e mira no hub
pela pose do Drive corrigida pela Limelight. Use `layouts/FRC9460-Shooter.json`
e o [guia do shooter](docs/SHOOTER.md).

```
config/advantagescope/
├── mecanismos/          modelos 3D lidos pelo AdvantageScope, um objeto por pasta
│   ├── Robot_Robo_9460/            robô: base + intake (model_0) + torre do shooter (model_1) + log
│   ├── Robot_Telescopio_9460/      telescópio 9460, etapa 1: N1 + N2 + N3 + log
│   └── Robot_Telescopio_9458/      telescópio 9458: tubo 4" + tubo 3" + tubo 2" + log
├── layouts/             arquivos para File > Import Layout
├── modelos_originais/   GLBs inteiros, de onde os mecanismos são gerados
│   ├── Robo_9460/
│   ├── Telescopio_9460/
│   └── Telescopio_9458/            original e versão agrupada por estágio (Blender)
├── scripts/             geradores Python (mecanismos, logs de demonstração e layouts)
├── docs/                guias
│   ├── MODELO_3D.md                modelos 3D, mecanismos e telescópio 9458
│   ├── CAMERAS.md                  teste das câmeras reais e virtuais
│   ├── SHOOTER.md                  shooter: zero manual, simulação e layout
│   ├── COMO_USAR.md                layout de visão
│   ├── VIRTUALIZACAO.md            simulação da visão
│   ├── REPLAY_VISAO.md             gravação e replay
│   ├── telescopio_9458/            telescópio 9458 no AdvantageScope
│   └── telescopio_9460_etapa1/     etapa 1 do telescópio 9460 (guia, imagens, validações)
├── analises/            análises de logs por data
└── backups/             layouts anteriores e cópias automáticas antes de mudanças
```

`mecanismos/` é a pasta de assets do AdvantageScope. Ela precisa estar
selecionada uma vez em **App > Use Custom Assets Folder...**; o guia dela é
[mecanismos/LEIA-ME.md](mecanismos/LEIA-ME.md).

## Ver os mecanismos se mexendo

1. **View > Reload** no AdvantageScope, depois de mudar algum GLB.
2. **File > Import Layout** > `layouts/FRC9460-Mecanismos.json` (só para
   analisar os mecanismos) ou `layouts/FRC9460-Simulacao3D.json` (completo).
3. **File > Open Log(s)** > o log do objeto, dentro da pasta dele em `mecanismos/`.
4. Abas de cada objeto (as primeiras do layout):
   - **Robo 9460** e **Robo 9460 3D**
   - **Telescopio 9460** e **Telescopio 9460 3D**
   - **Telescopio 9458** e **Telescopio 9458 3D**

As abas sem "3D" são desenhos 2D (Mechanism); as "3D" mostram os GLBs.
O telescópio 9460 também tem o layout próprio da etapa 1:
`layouts/Telescopio-Etapa1-Layout.json`.

## Regenerar (na pasta raiz do projeto)

| Para | Comando |
|---|---|
| Dividir o robô em base + mecanismos | `python config/advantagescope/scripts/preparar_componentes.py` |
| Dividir o telescópio 9458 (e o GLB por estágio) | `python config/advantagescope/scripts/preparar_telescopio_9458.py --agrupado` |
| Logs de demonstração (robô e telescópio 9458) | `python config/advantagescope/scripts/gerar_log_demo_componentes.py` |
| Layout só dos mecanismos | `python config/advantagescope/scripts/preparar_layout_mecanismos.py` |
| Layout de simulação 3D completo | `python config/advantagescope/scripts/preparar_simulacao_3d.py` |
| Layout de visão | `python config/advantagescope/scripts/preparar_layout.py` |
| Layout de teste das câmeras | `python config/advantagescope/scripts/preparar_layout_cameras.py` |
| Layout do shooter | `python config/advantagescope/scripts/preparar_layout_shooter.py` |
| Layout da simulação do telescópio 9458 | `python config/advantagescope/scripts/preparar_layout_telescopio_9458.py` |

O gerador do layout completo parte do estado salvo do AdvantageScope deste
computador (ou de outro layout, com `--estado`), preserva as abas existentes e
guarda uma cópia em `backups/layouts/`. O layout completo atual foi refeito a
partir de `backups/layouts/Layout-completo-2026-10-20-1136.json`. Os arquivos `.json` de registro dentro de `docs/` e
`backups/` mantêm os caminhos da época em que foram criados.
