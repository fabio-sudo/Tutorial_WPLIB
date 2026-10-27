"""Divide o telescópio da equipe 9458 em componentes do AdvantageScope.

Gera em mecanismos/Robot_Telescopio_9458:
  model.glb    tubo 4" (externo) e acionamento, fixo
  model_0.glb  tubo 3" (meio)
  model_1.glb  tubo 2" (interno)
  config.json  motor embaixo, base centrada no tubo externo, ponto mais baixo em Z = 0

O CAD foi exportado recolhido, então a extensão 0 é o telescópio fechado.
O movimento é só translação: a pose de cada tubo é (0, 0, altura), relativa
à base. O original modelos_originais/Telescopio_9458/Telescopio.glb não é alterado.

Com --agrupado, grava também modelos_originais/Telescopio_9458/
Telescopio_9458_por_estagio.glb:
o modelo inteiro com um grupo por estágio, para exportar cada um no Blender.
"""

import argparse
import json
from pathlib import Path

from preparar_componentes import Tree, extract, read_glb, write_glb

BASE = Path(__file__).resolve().parent.parent  # config/advantagescope
SOURCE = BASE / "modelos_originais" / "Telescopio_9458" / "Telescopio.glb"
OUTPUT = BASE / "mecanismos" / "Robot_Telescopio_9458"
GROUPED = SOURCE.with_name("Telescopio_9458_por_estagio.glb")
NAME = "FRC9458 - Telescopio"
OUTER = 'Tubo 4_x4_ MODIFICADO v1:1'  # perfil do tubo externo, usado para centralizar
MIDDLE = 'Tubo 3"x3":1'
INNER = 'Tubo 2"x2":1'
DRIVE = "Sistema de movimentação:1"
# Presilhas da corrente (tensores Spartan, bloco 61-01-04 e seus parafusos), na
# faixa Y do GLB onde ficam com o telescópio recolhido. Provavelmente puxam o
# estágio 2 pelo rasgo do tubo 4", mas ficam no estágio 1 porque as correntes,
# que não podem se deformar num modelo rígido, também ficam paradas nele.
REVIEW_Y = (-0.24, -0.17)


def stages(tree):
    """Nós de cada estágio; `review` é um subconjunto do estágio 1."""
    middle = set(tree.subtree(tree.one(MIDDLE)))
    inner = set(tree.subtree(tree.one(INNER)))
    fixed = set(range(len(tree.nodes))) - middle - inner
    review = set()
    for child in tree.nodes[tree.one(DRIVE)].get("children", []):
        for part in tree.nodes[child].get("children", []):
            nodes = tree.subtree(part)
            if not any("mesh" in tree.nodes[i] for i in nodes):
                continue
            lo, hi = tree.box(nodes)
            if lo[1] > REVIEW_Y[0] and hi[1] < REVIEW_Y[1]:
                review.update(nodes)
    return fixed, middle, inner, review


def write_grouped(path, document, binary, tree, groups):
    """GLB inteiro com um nó por grupo; (nome, nós, índice do grupo pai ou None)."""
    nodes, group_ids = [], []
    for label, owned, parent in groups:
        keep = set(owned)
        for i in owned:
            while i in tree.parent:
                i = tree.parent[i]
                keep.add(i)
        order = sorted(keep)
        index = {old: len(nodes) + k for k, old in enumerate(order)}
        for old in order:
            node = json.loads(json.dumps(document["nodes"][old]))
            children = [index[c] for c in node.pop("children", []) if c in keep]
            if children:
                node["children"] = children
            if "mesh" in node and old not in owned:
                del node["mesh"]  # ancestral só leva a transformação
            nodes.append(node)
        group_ids.append(len(nodes))
        nodes.append({"name": label, "children": [index[r] for r in tree.roots if r in keep]})
        if parent is not None:
            nodes[group_ids[parent]]["children"].append(group_ids[-1])
    result = dict(document, nodes=nodes, scene=0,
                  scenes=[{"name": "Telescopio 9458 por estagio",
                           "nodes": [g for g, (_, _, p) in zip(group_ids, groups) if p is None]}])
    write_glb(path, result, binary)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=SOURCE)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    parser.add_argument("--agrupado", action="store_true",
                        help="grava também o GLB inteiro agrupado por estágio")
    args = parser.parse_args()

    document, binary = read_glb(args.source)
    tree = Tree(document)
    base, middle, inner, review = stages(tree)

    # O motor fica na base. No GLB ele está em +Y e os tubos saem por -Y (a ponta
    # do tubo 2" aparece por esse lado com tudo recolhido). R_x(-90°) leva
    # (x, y, z) do GLB para (x, z, -y): motor embaixo e tubos saindo para +Z.
    outer_lo, outer_hi = tree.box(tree.subtree(tree.one(OUTER)))
    _, all_hi = tree.box(range(len(tree.nodes)))
    position = [round(v, 4) + 0.0 for v in (-(outer_lo[0] + outer_hi[0]) / 2,
                                            -(outer_lo[2] + outer_hi[2]) / 2, all_hi[1])]
    rotations = [{"axis": "x", "degrees": -90}]

    args.output.mkdir(parents=True, exist_ok=True)
    report = {"output": str(args.output), "files": {}}
    for name, owned, label in (("model.glb", base, 'tubo 4" + acionamento'),
                               ("model_0.glb", middle, 'tubo 3"'),
                               ("model_1.glb", inner, 'tubo 2"')):
        result, data, triangles = extract(document, binary, tree, owned, label)
        write_glb(args.output / name, result, data)
        report["files"][name] = {"part": label, "triangles": triangles,
                                 "megabytes": round((args.output / name).stat().st_size / 1e6, 2)}

    # Translação pura: o zero de cada tubo é a própria posição da base.
    component = {"zeroedRotations": rotations, "zeroedPosition": position}
    config = {"name": NAME, "isFTC": False, "disableSimplification": False,
              "rotations": rotations, "position": position, "cameras": [],
              "components": [component, component]}
    (args.output / "config.json").write_text(
        json.dumps(config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    report["position"] = position
    report["revisar"] = sorted(tree.nodes[i].get("name") for i in review if tree.parent.get(i) not in review)

    if args.agrupado:
        write_grouped(GROUPED, document, binary, tree, [
            ("Estagio1_Fixo_Tubo4", base - review, None),
            ("Estagio1_REVISAR_presilhas_corrente", review, 0),
            ("Estagio2_Tubo3", middle, None),
            ("Estagio3_Tubo2", inner, None)])
        report["agrupado"] = str(GROUPED)
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
