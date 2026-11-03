"""Extrai as posições de largada das bolas (Fuel) do campo 2026 do AdvantageScope.

Lê o modelo 3D do campo que o AdvantageScope baixa (autoAssets/Field3d_2026FRCFieldV2)
e grava src/main/deploy/simulacao/fuel_2026_inicial.json no referencial da WPILib
(origem azul, metros). A simulação das bolas começa com essas posições.

O AdvantageScope esconde as bolas do próprio campo quando o robô publica as dele,
por isso as bolas fora do campo (estação do jogador humano) também entram no arquivo.

Uso, na raiz do projeto: python config/advantagescope/scripts/extrair_bolas_campo_2026.py
"""

import json
import os
from pathlib import Path

from preparar_componentes import Tree, read_glb, transform

FIELD = Path(os.environ["APPDATA"]) / "AdvantageScope" / "autoAssets" / "Field3d_2026FRCFieldV2"
PROJECT = Path(__file__).resolve().parents[3]
OUTPUT = PROJECT / "src" / "main" / "deploy" / "simulacao" / "fuel_2026_inicial.json"
NODE = "GE-26900: Fuel"

# Centro do campo na WPILib. O campo do AdvantageScope usa a origem no centro e
# eixos girados 180° em relação à WPILib (conferido pelas posições das tags).
CENTER_X = 16.541 / 2
CENTER_Y = 8.069 / 2


def main():
    document, _ = read_glb(FIELD / "model.glb")
    tree = Tree(document)
    balls = []
    for index, node in enumerate(tree.nodes):
        if node.get("name") != NODE:
            continue
        x, y, z = transform(tree.world(index), [0, 0, 0])
        # glTF tem Y para cima: rotação de +90° em X leva para Z para cima.
        field_x, field_y, height = x, -z, y
        balls.append([round(CENTER_X - field_x, 3), round(CENTER_Y - field_y, 3), round(height, 3)])
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    # Uma bola por linha: fácil de ler e de comparar no Git.
    rows = ",\n".join("    " + json.dumps(ball) for ball in balls)
    OUTPUT.write_text(
        "{\n"
        '  "origem": "AdvantageScope autoAssets/Field3d_2026FRCFieldV2/model.glb",\n'
        '  "referencial": "WPILib, origem azul, metros: x, y e altura do centro",\n'
        f'  "bolas": [\n{rows}\n  ]\n'
        "}\n", encoding="utf-8")
    print(json.dumps({"arquivo": str(OUTPUT), "bolas": len(balls)}, ensure_ascii=False))


main()
