"""Retira a pose de cena do Unity sem modificar as malhas da montagem.

O asset do AdvantageScope aplica a posição/orientação do robô no campo.
Exportar também a posição do GameObject na cena soma um deslocamento indevido.
O arquivo de entrada nunca é sobrescrito; use uma saída intermediária.
"""

import argparse
import json
from pathlib import Path
import struct


def prepare(source: Path, output: Path):
    if source.resolve() == output.resolve():
        raise ValueError("Use uma saída diferente para preservar o original.")
    data = source.read_bytes()
    magic, version, length = struct.unpack_from("<4sII", data)
    if (magic, version, length) != (b"glTF", 2, len(data)):
        raise ValueError("Cabeçalho GLB 2.0 inválido.")

    chunks = []
    offset = 12
    while offset < len(data):
        size, kind = struct.unpack_from("<I4s", data, offset)
        offset += 8
        payload = data[offset:offset + size]
        if len(payload) != size or size % 4:
            raise ValueError("Chunk GLB incompleto ou desalinhado.")
        chunks.append((kind, payload))
        offset += size
    if offset != len(data) or chunks[0][0] != b"JSON":
        raise ValueError("Estrutura GLB inválida.")

    document = json.loads(chunks[0][1])
    if "Unity" not in document.get("asset", {}).get("generator", ""):
        raise ValueError("Este procedimento é específico para a exportação Unity.")
    roots = document["scenes"][document.get("scene", 0)].get("nodes", [])
    if len(roots) != 1 or document.get("animations"):
        raise ValueError("Revisar manualmente cenas com várias raízes ou animações.")
    root = document["nodes"][roots[0]]
    if "matrix" in root or any(abs(v - 1.0) > 1e-5 for v in root.get("scale", [1, 1, 1])):
        raise ValueError("Revisar manualmente a matriz/escala da raiz.")
    original_pose = {key: root.get(key) for key in ("translation", "rotation", "scale")}
    # Somente o GameObject raiz perde a pose de mundo. Todos os filhos mantêm
    # montagens, ângulos, escalas, materiais e hierarquia da exportação original.
    root.pop("translation", None)
    root.pop("rotation", None)

    payload = json.dumps(document, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    payload += b" " * (-len(payload) % 4)
    chunks[0] = (b"JSON", payload)
    body = b"".join(struct.pack("<I4s", len(part), kind) + part for kind, part in chunks)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(struct.pack("<4sII", b"glTF", 2, len(body) + 12) + body)
    print(json.dumps({"source": str(source), "output": str(output),
        "root": root.get("name"), "removedWorldPose": original_pose,
        "binaryChunksPreserved": True}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    prepare(arguments.source, arguments.output)
