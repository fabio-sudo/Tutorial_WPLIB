# Mecanismos

Esta é a pasta de modelos 3D do AdvantageScope. Cada subpasta é um objeto,
com os GLBs já divididos e um log de demonstração para ver o movimento.

| Pasta | Nome no AdvantageScope | `model.glb` (fixo) | `model_0.glb` | `model_1.glb` |
|---|---|---|---|---|
| `Robot_Robo_9460` | FRC9460 - Robo 2026 | chassi, swerve, base do shooter | intake (sobe/desce) | torre do shooter (gira) |
| `Robot_Telescopio_9460` | FRC9460 - Telescopio - Etapa 1 | N1 | N2 | N3 |
| `Robot_Telescopio_9458` | FRC9458 - Telescopio | tubo 4" + acionamento | tubo 3" | tubo 2" |

| Objeto | Log de demonstração | Abas do layout |
|---|---|---|
| Robô 9460 | `Robot_Robo_9460/Robo_9460-Demo.wpilog` | **Robo 9460** (2D) e **Robo 9460 3D** |
| Telescópio 9460 | `Robot_Telescopio_9460/Telescopio_9460-Demo.wpilog` | **Telescopio 9460** (2D) e **Telescopio 9460 3D** |
| Telescópio 9458 | `Robot_Telescopio_9458/Telescopio_9458-Demo.wpilog` | **Telescopio 9458** (2D) e **Telescopio 9458 3D** |

As abas sem "3D" são do tipo Mechanism (⚙️): desenham linhas 2D a partir do
log e não usam os GLBs. As abas "3D" mostram os GLBs se mexendo.

## Como ver

1. Uma vez: **App > Use Custom Assets Folder...** e escolha esta pasta
   (`config/advantagescope/mecanismos`).
2. **View > Reload**, sempre que um GLB mudar.
3. **File > Import Layout...** > `../layouts/FRC9460-Mecanismos.json` (só as
   abas dos objetos e gráficos dos movimentos). O layout completo
   `../layouts/FRC9460-Simulacao3D.json` também tem essas abas.
4. **File > Open Log(s)...** > o log do objeto (tabela acima).
5. Abra as duas abas do objeto e use a reprodução ou clique na linha do tempo.

## Movimentos dos logs

- **Robô 9460** (24 s): intake de 0° (abaixado) a 90° (recolhido) a cada 8 s;
  torre de −90° a +90° (limite da fiação) a cada 12 s. No 2D, o intake
  aparece visto de lado; a torre só aparece no 3D.
- **Telescópio 9460** (20 s): etapa 1, já conferida (veja
  `../docs/telescopio_9460_etapa1/ETAPA_1.md`).
- **Telescópio 9458** (15 s): cascata, os dois estágios sobem juntos. Para
  em 10 cm + 10 cm (ponta a 0,2 m), sobe até o fim de curso de 78,4 voltas
  do motor (0,498 m por estágio, 0,996 m na ponta, altura ~1,776 m) e fecha. Parâmetros em
  `../docs/telescopio_9458/ADVANTAGESCOPE.md`; os tempos são de demonstração.

## Regras desta pasta

- Cada subpasta precisa começar com `Robot_`; outro nome gera aviso de falha
  ao carregar os assets.
- O AdvantageScope lê só `config.json`, `model.glb` e `model_N.glb`; logs e
  este arquivo são ignorados por ele.
- Não troque o `name` do `config.json`: os layouts procuram o modelo por esse nome.
- Os GLBs são gerados pelos scripts de `../scripts` a partir de
  `../modelos_originais`; veja `../docs/MODELO_3D.md`.
