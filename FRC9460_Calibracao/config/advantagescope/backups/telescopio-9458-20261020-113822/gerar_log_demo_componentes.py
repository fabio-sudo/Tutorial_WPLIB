"""Gera um .wpilog de demonstração dos componentes do robô (sem código Java).

O log imita o formato do AdvantageKit e publica:
  /RealOutputs/Robot/Pose                 Pose2d parado no campo
  /RealOutputs/Mecanismos/Componentes     Pose3d[] {intake, torre do shooter}
  /RealOutputs/Mecanismos/IntakeGraus     ângulo de subida do intake
  /RealOutputs/Mecanismos/TorreGraus      ângulo da torre (anti-horário visto de cima)
  /RealOutputs/Telescopio9458/Componentes Pose3d[] {tubo 3", tubo 2"} do telescópio 9458
  /RealOutputs/Telescopio9458/ExtensaoMetros  altura da ponta acima da posição recolhida

As poses usam a mesma convenção que o código Java deverá usar depois:
posição do pivô no robô (config.position - zeroedPosition) + rotação da junta.
Abra o arquivo com File > Open Log(s) e use as abas SIM 3D - Mecanismos e
SIM 3D - Telescopio 9458.
"""

import argparse
import json
import math
from pathlib import Path
import struct

HERE = Path(__file__).resolve().parent
CONFIG = HERE / "assets" / "Robot_FRC9460_Calibracao" / "config.json"
OUTPUT = HERE / "demo" / "FRC9460-Componentes-Demo.wpilog"
ROOT = "/RealOutputs/"
META = '{"source":"AdvantageKit"}'
SCHEMAS = {
    "Translation2d": "double x;double y",
    "Rotation2d": "double value",
    "Pose2d": "Translation2d translation;Rotation2d rotation",
    "Translation3d": "double x;double y;double z",
    "Quaternion": "double w;double x;double y;double z",
    "Rotation3d": "Quaternion q",
    "Pose3d": "Translation3d translation;Rotation3d rotation",
}
ROBOT_POSE = (2.5, 4.0, 0.0)  # metros, metros, radianos
TURRET_LIMIT_DEGREES = 90.0  # limite da fiação para cada lado
TELESCOPE_STAGE_TRAVEL = 0.55  # m por estágio; palpite que mantém os tubos encaixados
PERIOD = 0.02
DURATION = 24.0


class Writer:
    """Registros com id de 4 bytes, tamanho de 4 bytes e timestamp de 8 bytes."""

    def __init__(self):
        extra = b"AdvantageKit"
        self.data = bytearray(b"WPILOG" + struct.pack("<HI", 0x0100, len(extra)) + extra)
        self.ids = {}

    def record(self, entry, micros, payload):
        self.data += bytes([0x7F]) + struct.pack("<IIQ", entry, len(payload), micros) + payload

    def start(self, name, kind, micros=0):
        entry = self.ids[name] = len(self.ids) + 1
        fields = [name.encode(), kind.encode(), META.encode()]
        self.record(0, micros, b"\0" + struct.pack("<I", entry)
                    + b"".join(struct.pack("<I", len(f)) + f for f in fields))

    def append(self, name, micros, payload):
        self.record(self.ids[name], micros, payload)


def pose3d(translation, quaternion):
    return struct.pack("<7d", *translation, *quaternion)


def axis_quaternion(axis, radians):
    s = math.sin(radians / 2)
    return (math.cos(radians / 2), *(s if a == axis else 0.0 for a in "xyz"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()

    config = json.loads(CONFIG.read_text(encoding="utf-8"))
    if len(config["components"]) != 2:
        raise ValueError("Esperados 2 componentes (intake e torre). Rode preparar_componentes.py.")
    pivots = [[o - z for o, z in zip(config["position"], c["zeroedPosition"])]
              for c in config["components"]]

    log = Writer()
    log.start("/Timestamp", "int64")
    for name, schema in SCHEMAS.items():
        log.start(f"/.schema/struct:{name}", "structschema")
        log.append(f"/.schema/struct:{name}", 0, schema.encode())
    log.start(ROOT + "Robot/Pose", "struct:Pose2d")
    log.start(ROOT + "Mecanismos/Componentes", "struct:Pose3d[]")
    log.start(ROOT + "Mecanismos/IntakeGraus", "double")
    log.start(ROOT + "Mecanismos/TorreGraus", "double")
    log.start(ROOT + "Telescopio9458/Componentes", "struct:Pose3d[]")
    log.start(ROOT + "Telescopio9458/ExtensaoMetros", "double")

    for step in range(int(DURATION / PERIOD) + 1):
        t = step * PERIOD
        micros = round(t * 1e6)
        # Intake: 0° (abaixado, como no CAD) até 90° (recolhido), ciclo de 8 s.
        intake = 45.0 * (1 - math.cos(2 * math.pi * t / 8.0))
        # Torre: varre até o limite da fiação (±90°) em 12 s.
        turret = TURRET_LIMIT_DEGREES * math.sin(2 * math.pi * t / 12.0)
        components = (
            # Pitch negativo levanta a ponta do intake (rotação em torno de +Y).
            pose3d(pivots[0], axis_quaternion("y", -math.radians(intake)))
            + pose3d(pivots[1], axis_quaternion("z", math.radians(turret))))
        log.append("/Timestamp", micros, struct.pack("<q", micros))
        log.append(ROOT + "Robot/Pose", micros, struct.pack("<3d", *ROBOT_POSE))
        log.append(ROOT + "Mecanismos/Componentes", micros, components)
        log.append(ROOT + "Mecanismos/IntakeGraus", micros, struct.pack("<d", intake))
        log.append(ROOT + "Mecanismos/TorreGraus", micros, struct.pack("<d", turret))
        # Telescópio 9458: os dois estágios sobem juntos e voltam, ciclo de 12 s.
        # Poses relativas à base: o tubo 2" soma a extensão do tubo 3".
        stage = TELESCOPE_STAGE_TRAVEL * 0.5 * (1 - math.cos(2 * math.pi * t / 12.0))
        no_rotation = (1.0, 0.0, 0.0, 0.0)
        log.append(ROOT + "Telescopio9458/Componentes", micros,
                   pose3d((0.0, 0.0, stage), no_rotation) + pose3d((0.0, 0.0, 2 * stage), no_rotation))
        log.append(ROOT + "Telescopio9458/ExtensaoMetros", micros, struct.pack("<d", 2 * stage))

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(log.data)
    print(json.dumps({"output": str(args.output), "seconds": DURATION,
                      "pivotsRobotFrameMeters": {"intake": [round(v, 4) + 0.0 for v in pivots[0]],
                                                 "torre": [round(v, 4) + 0.0 for v in pivots[1]]}},
                     ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
