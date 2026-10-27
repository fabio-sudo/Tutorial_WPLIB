"""Divide o modelo inteiro do robô em componentes articulados do AdvantageScope.

Gera, na pasta do asset:
  model.glb    parte fixa (chassi, swerve, centrífuga, base do shooter e
               motores/mancais do pivô do intake)
  model_0.glb  intake (braços, rolo, proteções) que gira no pivô
  model_1.glb  torre do shooter (parte superior sobre a engrenagem grande)
  config.json  com "components": cada componente tem o pivô na origem

As malhas não são alteradas: só a árvore de nós é separada e cada arquivo
leva apenas os dados binários que usa. O modelo inteiro nunca é sobrescrito;
ele fica em modelos_originais/ (fora de mecanismos/, para não duplicar o robô).
"""

import argparse
import json
import math
from pathlib import Path
import struct

BASE = Path(__file__).resolve().parent.parent  # config/advantagescope
SOURCE = BASE / "modelos_originais" / "Robo_9460" / "model.glb"
SOURCE_CONFIG = SOURCE.with_name("config.json")
OUTPUT = BASE / "mecanismos" / "Robot_Robo_9460"

# Nós da exportação Unity/Blender (nomes vindos do CAD).
INTAKE = "Intake"
INTAKE_FIXED = "Montagem mancal - dir:"  # Kraken X44 + redução + mancal, presos ao chassi
INTAKE_BEARING = "Rolamento"            # rolamento do pivô dentro de INTAKE_FIXED
TURRET = "ShooterPivot"
TURRET_GEAR = "Spur Gear2"              # engrenagem horizontal da torre
REMOVED = ("IntakeGrabHandle",)         # cubo verde translúcido de VR do Unity


def read_glb(path):
    data = path.read_bytes()
    magic, version, length = struct.unpack_from("<4sII", data)
    if (magic, version, length) != (b"glTF", 2, len(data)):
        raise ValueError(f"Cabeçalho GLB 2.0 inválido: {path}")
    chunks, offset = [], 12
    while offset < len(data):
        size, kind = struct.unpack_from("<I4s", data, offset)
        chunks.append((kind, data[offset + 8:offset + 8 + size]))
        offset += 8 + size
    if chunks[0][0] != b"JSON" or len(chunks) != 2 or chunks[1][0] != b"BIN\0":
        raise ValueError("Esperado GLB com um chunk JSON e um chunk BIN.")
    document = json.loads(chunks[0][1])
    if len(document.get("buffers", [])) != 1 or document.get("extensionsRequired"):
        raise ValueError("Revisar manualmente: vários buffers ou extensões obrigatórias.")
    if document.get("animations") or document.get("skins"):
        raise ValueError("Revisar manualmente: o GLB contém animações ou skins.")
    return document, chunks[1][1]


def write_glb(path, document, binary):
    payload = json.dumps(document, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    payload += b" " * (-len(payload) % 4)
    binary += b"\0" * (-len(binary) % 4)
    body = (struct.pack("<I4s", len(payload), b"JSON") + payload
            + struct.pack("<I4s", len(binary), b"BIN\0") + binary)
    path.write_bytes(struct.pack("<4sII", b"glTF", 2, len(body) + 12) + body)


# --- Transformações (matrizes 4x4 em listas, coluna = eixo local) ----------

def node_matrix(node):
    if "matrix" in node:
        m = node["matrix"]
        return [[m[c * 4 + r] for c in range(4)] for r in range(4)]
    x, y, z, w = node.get("rotation", [0, 0, 0, 1])
    sx, sy, sz = node.get("scale", [1, 1, 1])
    tx, ty, tz = node.get("translation", [0, 0, 0])
    r = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
         [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
         [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    return [[r[i][0] * sx, r[i][1] * sy, r[i][2] * sz, t] for i, t in enumerate((tx, ty, tz))] \
        + [[0, 0, 0, 1]]


def multiply(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def transform(m, p):
    return [sum(m[i][k] * p[k] for k in range(3)) + m[i][3] for i in range(3)]


class Tree:
    def __init__(self, document):
        self.doc = document
        self.nodes = document["nodes"]
        self.parent = {c: i for i, n in enumerate(self.nodes) for c in n.get("children", [])}
        self.roots = document["scenes"][document.get("scene", 0)]["nodes"]

    def find(self, prefix, inside=None):
        scope = self.subtree(inside) if inside is not None else range(len(self.nodes))
        return [i for i in scope if self.nodes[i].get("name", "").startswith(prefix)]

    def one(self, name):
        found = [i for i, n in enumerate(self.nodes) if n.get("name") == name]
        if len(found) != 1:
            raise ValueError(f"Esperado um nó '{name}', encontrados {len(found)}.")
        return found[0]

    def subtree(self, index):
        result, stack = [], [index]
        while stack:
            i = stack.pop()
            result.append(i)
            stack.extend(self.nodes[i].get("children", []))
        return result

    def world(self, index):
        chain = []
        while index is not None:
            chain.append(index)
            index = self.parent.get(index)
        m = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
        for i in reversed(chain):
            m = multiply(m, node_matrix(self.nodes[i]))
        return m

    def box(self, indices):
        """Caixa (coordenadas do GLB) das malhas dos nós indicados."""
        lo, hi = [math.inf] * 3, [-math.inf] * 3
        for i in indices:
            if "mesh" not in self.nodes[i]:
                continue
            m = self.world(i)
            for prim in self.doc["meshes"][self.nodes[i]["mesh"]]["primitives"]:
                acc = self.doc["accessors"][prim["attributes"]["POSITION"]]
                for corner in range(8):
                    p = transform(m, [(acc["max"] if corner >> k & 1 else acc["min"])[k]
                                      for k in range(3)])
                    lo = [min(a, b) for a, b in zip(lo, p)]
                    hi = [max(a, b) for a, b in zip(hi, p)]
        if lo[0] == math.inf:
            raise ValueError("Nenhuma malha encontrada para medir o pivô.")
        return lo, hi


def extract(document, binary, tree, owned, label):
    """Cria um GLB com as malhas de `owned`; ancestrais entram só como transformação."""
    keep = set(owned)
    for i in owned:
        while i in tree.parent:
            i = tree.parent[i]
            keep.add(i)
    order = sorted(keep)
    new_node = {old: new for new, old in enumerate(order)}
    used = {"meshes": {}, "accessors": {}, "bufferViews": {}, "materials": {},
            "textures": {}, "images": {}, "samplers": {}}

    def ref(kind, old):
        return used[kind].setdefault(old, len(used[kind]))

    def remap_textures(value):
        # textureInfo: qualquer objeto "...Texture" com "index" (inclui extensões).
        if isinstance(value, dict):
            for key, item in value.items():
                if key.endswith("Texture") and isinstance(item, dict) and "index" in item:
                    item["index"] = ref("textures", item["index"])
                remap_textures(item)
        elif isinstance(value, list):
            for item in value:
                remap_textures(item)

    nodes = []
    for old in order:
        node = json.loads(json.dumps(document["nodes"][old]))
        children = [new_node[c] for c in node.pop("children", []) if c in keep]
        if children:
            node["children"] = children
        if "mesh" in node:
            if old in owned:
                node["mesh"] = ref("meshes", node["mesh"])
            else:
                del node["mesh"]
        nodes.append(node)

    # A ordem de inserção de `used[kind]` é o novo índice. Cada etapa só cria
    # referências de tipos copiados depois dela (malha -> acessor -> bufferView).
    out = {key: [] for key in used}

    def copy_in_order(kind, fix):
        for old in list(used[kind]):
            item = json.loads(json.dumps(document[kind][old]))
            fix(item)
            out[kind].append(item)

    def fix_mesh(mesh):
        for prim in mesh["primitives"]:
            prim["attributes"] = {k: ref("accessors", v) for k, v in prim["attributes"].items()}
            if "indices" in prim:
                prim["indices"] = ref("accessors", prim["indices"])
            if "material" in prim:
                prim["material"] = ref("materials", prim["material"])
            for target in prim.get("targets", []):
                for k in target:
                    target[k] = ref("accessors", target[k])
    copy_in_order("meshes", fix_mesh)
    copy_in_order("materials", remap_textures)

    def fix_texture(item):
        if "source" in item:
            item["source"] = ref("images", item["source"])
        if "sampler" in item:
            item["sampler"] = ref("samplers", item["sampler"])
    copy_in_order("textures", fix_texture)

    def fix_view_ref(item):
        if "bufferView" in item:
            item["bufferView"] = ref("bufferViews", item["bufferView"])
    copy_in_order("accessors", fix_view_ref)
    copy_in_order("images", fix_view_ref)
    copy_in_order("samplers", lambda item: None)

    # Novo buffer apenas com as bufferViews usadas, alinhadas a 4 bytes.
    data = bytearray()
    for old in used["bufferViews"]:
        view = dict(document["bufferViews"][old])
        start = view.get("byteOffset", 0)
        data += b"\0" * (-len(data) % 4)
        view["byteOffset"] = len(data)
        view["buffer"] = 0
        data += binary[start:start + view["byteLength"]]
        out["bufferViews"].append(view)

    result = {"asset": dict(document["asset"], extras={"FRC9460": label}),
              "scene": 0,
              "scenes": [{"name": label, "nodes": [new_node[r] for r in tree.roots if r in keep]}],
              "nodes": nodes,
              "buffers": [{"byteLength": len(data)}]}
    for kind in ("meshes", "materials", "accessors", "bufferViews", "textures", "images", "samplers"):
        if out[kind]:
            result[kind] = out[kind]
    triangles = sum(document["accessors"][p["indices"]]["count"] // 3
                    for i in owned if "mesh" in document["nodes"][i]
                    for p in document["meshes"][document["nodes"][i]["mesh"]]["primitives"]
                    if "indices" in p)
    return result, bytes(data), triangles


def robot_frame(config, point):
    """Converte um ponto do GLB para o referencial do robô usado pelo config.json."""
    p = list(point)
    for step in config["rotations"]:
        a = math.radians(step["degrees"])
        c, s = math.cos(a), math.sin(a)
        x, y, z = p
        p = {"x": [x, c * y - s * z, s * y + c * z],
             "y": [c * x + s * z, y, -s * x + c * z],
             "z": [c * x - s * y, s * x + c * y, z]}[step["axis"]]
    return [v + o for v, o in zip(p, config["position"])]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=SOURCE)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()
    if args.source.resolve().parent == args.output.resolve():
        raise ValueError("A origem deve ser o modelo inteiro guardado fora da pasta de saída.")

    document, binary = read_glb(args.source)
    config = json.loads(args.source.with_name("config.json").read_text(encoding="utf-8"))
    tree = Tree(document)

    removed = {i for name in REMOVED for root in tree.find(name) for i in tree.subtree(root)}
    intake_root = tree.one(INTAKE)
    fixed_roots = tree.find(INTAKE_FIXED, inside=intake_root)
    fixed = {i for root in fixed_roots for i in tree.subtree(root)}
    intake = set(tree.subtree(intake_root)) - fixed - removed
    turret_root = tree.one(TURRET)
    turret = set(tree.subtree(turret_root)) - removed
    base = set(range(len(tree.nodes))) - intake - turret - removed
    if len(fixed_roots) != 2:
        raise ValueError(f"Esperados 2 conjuntos '{INTAKE_FIXED}', encontrados {len(fixed_roots)}.")

    # Pivô do intake: centro dos dois rolamentos do eixo (um de cada lado).
    centers = []
    for root in fixed_roots:
        bearings = tree.find(INTAKE_BEARING, inside=root)
        lo, hi = tree.box([i for b in bearings for i in tree.subtree(b)])
        centers.append([(a + b) / 2 for a, b in zip(lo, hi)])
    intake_pivot = [sum(c[k] for c in centers) / 2 for k in range(3)]
    # Pivô da torre: centro da engrenagem horizontal, na face inferior dela.
    gear = tree.find(TURRET_GEAR, inside=turret_root)
    lo, hi = tree.box([i for g in gear for i in tree.subtree(g)])
    turret_pivot = [(lo[0] + hi[0]) / 2, lo[1], (lo[2] + hi[2]) / 2]

    args.output.mkdir(parents=True, exist_ok=True)
    report = {"source": str(args.source), "output": str(args.output), "files": {}}
    pieces = [("model.glb", base, "base fixa"), ("model_0.glb", intake, "intake"),
              ("model_1.glb", turret, "torre do shooter")]
    for name, owned, label in pieces:
        result, data, triangles = extract(document, binary, tree, owned, label)
        write_glb(args.output / name, result, data)
        report["files"][name] = {"part": label, "meshNodes": sum("mesh" in n for n in result["nodes"]),
                                 "triangles": triangles,
                                 "megabytes": round((args.output / name).stat().st_size / 1e6, 2)}

    components, pivots = [], {}
    for label, point in (("intake", intake_pivot), ("torre do shooter", turret_pivot)):
        pivot = robot_frame(config, point)
        # zeroed: mesma rotação do robô e pivô levado à origem. A pose registrada
        # no código é então (posição do pivô no robô, rotação da junta).
        components.append({"zeroedRotations": config["rotations"],
                           "zeroedPosition": [round(o - v, 4) for o, v in
                                              zip(config["position"], pivot)]})
        pivots[label] = [round(v, 4) + 0.0 for v in pivot]
    output_config = dict(config, components=components)
    (args.output / "config.json").write_text(
        json.dumps(output_config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    report["pivotsRobotFrameMeters"] = pivots
    report["removed"] = sorted({tree.nodes[i].get("name") for i in removed})
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
