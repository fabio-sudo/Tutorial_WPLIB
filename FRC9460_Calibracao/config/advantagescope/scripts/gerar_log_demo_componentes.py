"""Gera um .wpilog de demonstração por objeto em mecanismos/ (sem código Java).

Os logs imitam o formato do AdvantageKit e são gravados ao lado dos GLBs:

mecanismos/Robot_Robo_9460/Robo_9460-Demo.wpilog
  /RealOutputs/Robot/Pose                    Pose2d na origem
  /RealOutputs/Mecanismos/Componentes        Pose3d[] {intake, torre do shooter}
  /RealOutputs/Mecanismos/IntakeGraus        ângulo de subida do intake
  /RealOutputs/Mecanismos/TorreGraus         ângulo da torre (anti-horário visto de cima)
  /RealOutputs/Mecanismos/Mecanismo2d        intake visto de lado (aba Mechanism)

mecanismos/Robot_Telescopio_9458/Telescopio_9458-Demo.wpilog
  /RealOutputs/Telescopio9458/Pose           Pose3d da base, na origem
  /RealOutputs/Telescopio9458/Componentes    Pose3d[] {tubo 3", tubo 2"}
  /RealOutputs/Telescopio9458/ExtensaoTubo3Metros, ExtensaoTubo2Metros, ExtensaoMetros
  /RealOutputs/Telescopio9458/MotorRotacoes, AlturaTotalMetros
  /RealOutputs/Telescopio9458/Mecanismo2d    tubos vistos de lado (aba Mechanism)

As poses usam a convenção que o código Java deverá usar depois. Abra um log
com File > Open Log(s) e use as abas do objeto (ex.: Telescopio 9458 e
Telescopio 9458 3D) do layout layouts/FRC9460-Simulacao3D.json.
"""

import argparse
import json
import math
from pathlib import Path
import struct

from preparar_componentes import Tree, read_glb, robot_frame

BASE = Path(__file__).resolve().parent.parent  # config/advantagescope
ROBOT = BASE / "mecanismos" / "Robot_Robo_9460"
TELESCOPE = BASE / "mecanismos" / "Robot_Telescopio_9458"
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
PERIOD = 0.02
NO_ROTATION = (1.0, 0.0, 0.0, 0.0)

# Robô 9460
ROBOT_DURATION = 24.0
INTAKE_UP_DEGREES = 90.0     # recolhido; 0° = abaixado, como no CAD
TURRET_LIMIT_DEGREES = 90.0  # limite da fiação para cada lado

# Telescópio 9458 — os mesmos valores de TelescopioConstants_9458 (Java).
# Redução e fim de curso do robô físico; coroa e altura fechado do CAD.
CLOSED_HEIGHT = 0.7802            # m, topo do tubo 2" no CAD (a equipe mediu 0,80 m)
MOTOR_ROTATIONS_AT_LIMIT = 78.4   # voltas do rotor do Kraken X60 até o fim de curso (medido)
GEAR_REDUCTION = 16.0             # duas reduções de 4:1 (o redutor do CAD é 12:1)
SPROCKET_TEETH = 16               # coroa REV-21-2016 do CAD
CHAIN_PITCH = 0.00635             # m, corrente #25
SPROCKET_PITCH_DIAMETER = CHAIN_PITCH / math.sin(math.pi / SPROCKET_TEETH)  # 32,55 mm
MASS_KG = 5.0                     # para a simulação física (Java); não afeta a demo
# A corrente anda 16 × 6,35 = 101,6 mm por volta da coroa: 6,35 mm por volta do motor.
METERS_PER_MOTOR_ROTATION = SPROCKET_TEETH * CHAIN_PITCH / GEAR_REDUCTION
# Cascata: o tubo 2" sobe em relação ao tubo 3" o mesmo que o tubo 3" sobe
# em relação ao tubo 4" (10 cm + 10 cm). Curso por estágio = 78,4 × 6,35 mm ≈ 0,498 m
# e deslocamento da ponta ≈ 0,996 m (0,780 m + 0,996 m ≈ 1,776 m aberto).
STAGE_TRAVEL = MOTOR_ROTATIONS_AT_LIMIT * METERS_PER_MOTOR_ROTATION
FIRST_STOP = 0.10    # m por estágio: parada para mostrar 10 cm + 10 cm
MOVE_SECONDS = 3.0   # demonstração lenta; a velocidade real não foi informada
HOLD_SECONDS = 1.5   # pausa entre movimentos


class Writer:
    """Registros com id de 4 bytes, tamanho de 4 bytes e timestamp de 8 bytes."""

    def __init__(self):
        extra = b"AdvantageKit"
        self.data = bytearray(b"WPILOG" + struct.pack("<HI", 0x0100, len(extra)) + extra)
        self.ids = {}
        self.start("/Timestamp", "int64")
        for name, schema in SCHEMAS.items():
            self.constant(f"/.schema/struct:{name}", "structschema", schema.encode())

    def record(self, entry, micros, payload):
        self.data += bytes([0x7F]) + struct.pack("<IIQ", entry, len(payload), micros) + payload

    def start(self, name, kind):
        entry = self.ids[name] = len(self.ids) + 1
        fields = [name.encode(), kind.encode(), META.encode()]
        self.record(0, 0, b"\0" + struct.pack("<I", entry)
                    + b"".join(struct.pack("<I", len(f)) + f for f in fields))

    def append(self, name, micros, payload):
        self.record(self.ids[name], micros, payload)

    def constant(self, name, kind, payload):
        self.start(name, kind)
        self.append(name, 0, payload)

    def tick(self, micros):
        self.append("/Timestamp", micros, struct.pack("<q", micros))

    def save(self, path):
        path.write_bytes(self.data)
        return str(path)


def pose3d(translation, quaternion):
    return struct.pack("<7d", *translation, *quaternion)


def axis_quaternion(axis, radians):
    s = math.sin(radians / 2)
    return (math.cos(radians / 2), *(s if a == axis else 0.0 for a in "xyz"))


def double(value):
    return struct.pack("<d", value)


def mechanism_line(log, path, angle, length, color, weight):
    """Segmento fixo de um Mechanism2d (formato do LoggedMechanism2d)."""
    log.constant(path + "/.type", "string", b"line")
    log.constant(path + "/length", "double", double(length))
    log.constant(path + "/color", "string", color.encode())
    log.constant(path + "/weight", "double", double(weight))
    if angle is None:
        log.start(path + "/angle", "double")  # varia no tempo
    else:
        log.constant(path + "/angle", "double", double(angle))


def mechanism_canvas(log, key, width, height):
    log.constant(key + "/.type", "string", b"Mechanism2d")
    log.constant(key + "/backgroundColor", "string", b"#202934")
    log.constant(key + "/dims", "double[]", struct.pack("<2d", width, height))


def robot_log():
    config = json.loads((ROBOT / "config.json").read_text(encoding="utf-8"))
    if len(config["components"]) != 2:
        raise ValueError("Esperados 2 componentes (intake e torre). Rode preparar_componentes.py.")
    pivots = [[o - z for o, z in zip(config["position"], c["zeroedPosition"])]
              for c in config["components"]]

    # Braço do intake no desenho 2D: do pivô até o eixo do rolo (medido no GLB).
    document, _ = read_glb(ROBOT / "model_0.glb")
    tree = Tree(document)
    lo, hi = tree.box(tree.subtree(tree.one("IntakeRoller")))
    roller = [(a + b) / 2 for a, b in zip(lo, hi)]
    roller_x, roller_z = roller[0] + config["position"][0], roller[1] + config["position"][2]
    arm = math.hypot(roller_x - pivots[0][0], roller_z - pivots[0][2])
    arm_angle = math.degrees(math.atan2(roller_z - pivots[0][2], roller_x - pivots[0][0]))

    log = Writer()
    log.start(ROOT + "Robot/Pose", "struct:Pose2d")
    log.start(ROOT + "Mecanismos/Componentes", "struct:Pose3d[]")
    log.start(ROOT + "Mecanismos/IntakeGraus", "double")
    log.start(ROOT + "Mecanismos/TorreGraus", "double")
    # Vista lateral: X do robô para a direita, origem do desenho 0,15 m antes do para-choque.
    key = ROOT + "Mecanismos/Mecanismo2d"
    offset_x, offset_z = 0.419 + 0.15, 0.05
    mechanism_canvas(log, key, 1.4, 0.9)
    log.constant(key + "/Chassi/x", "double", double(0.15))
    log.constant(key + "/Chassi/y", "double", double(offset_z + 0.06))
    mechanism_line(log, key + "/Chassi/ParaChoque", 0.0, 0.838, "#3366ff", 14.0)
    log.constant(key + "/Intake/x", "double", double(pivots[0][0] + offset_x))
    log.constant(key + "/Intake/y", "double", double(pivots[0][2] + offset_z))
    mechanism_line(log, key + "/Intake/Braco", None, arm, "#ff8c00", 8.0)

    for step in range(int(ROBOT_DURATION / PERIOD) + 1):
        t = step * PERIOD
        micros = round(t * 1e6)
        intake = INTAKE_UP_DEGREES / 2 * (1 - math.cos(2 * math.pi * t / 8.0))
        turret = TURRET_LIMIT_DEGREES * math.sin(2 * math.pi * t / 12.0)
        log.tick(micros)
        log.append(ROOT + "Robot/Pose", micros, struct.pack("<3d", 0.0, 0.0, 0.0))
        log.append(ROOT + "Mecanismos/Componentes", micros,
                   # Pitch negativo levanta a ponta do intake (rotação em torno de +Y).
                   pose3d(pivots[0], axis_quaternion("y", -math.radians(intake)))
                   + pose3d(pivots[1], axis_quaternion("z", math.radians(turret))))
        log.append(ROOT + "Mecanismos/IntakeGraus", micros, double(intake))
        log.append(ROOT + "Mecanismos/TorreGraus", micros, double(turret))
        log.append(key + "/Intake/Braco/angle", micros, double(arm_angle + intake))
    return log.save(ROBOT / "Robo_9460-Demo.wpilog")


def smooth(x):
    """0 → 1 com início e fim suaves."""
    x = min(max(x, 0.0), 1.0)
    return 0.5 * (1 - math.cos(math.pi * x))


def telescope_steps():
    """Roteiro do curso por estágio: 10 cm, curso completo, volta a fechar."""
    targets = [(0.0, FIRST_STOP), (FIRST_STOP, STAGE_TRAVEL), (STAGE_TRAVEL, 0.0)]
    t, plan = HOLD_SECONDS, []
    for start, end in targets:
        plan.append((t, start, end))
        t += MOVE_SECONDS + HOLD_SECONDS
    return plan, t


def telescope_extension(t, plan):
    """Extensão de cada estágio (m); os dois estágios andam juntos."""
    stage = 0.0
    for begin, start, end in plan:
        if t >= begin:
            stage = start + (end - start) * smooth((t - begin) / MOVE_SECONDS)
    return stage


def telescope_log():
    config = json.loads((TELESCOPE / "config.json").read_text(encoding="utf-8"))
    log = Writer()
    log.start(ROOT + "Telescopio9458/Pose", "struct:Pose3d")
    log.start(ROOT + "Telescopio9458/Componentes", "struct:Pose3d[]")
    for name in ("ExtensaoTubo3Metros", "ExtensaoTubo2Metros", "ExtensaoMetros",
                 "MotorRotacoes", "AlturaTotalMetros"):
        log.start(ROOT + "Telescopio9458/" + name, "double")
    # Movimento com curso real, mas tempos de demonstração (sem física).
    log.constant(ROOT + "Telescopio9458/Modo", "string", b"DEMONSTRACAO_VISUAL_SEM_FISICA")
    log.constant(ROOT + "Telescopio9458/CursoRealConfirmado", "boolean", b"\x01")
    for name, value in (("CursoPorEstagioMetros", STAGE_TRAVEL),
                        ("MotorRotacoesNoLimite", MOTOR_ROTATIONS_AT_LIMIT),
                        ("ReducaoTotal", GEAR_REDUCTION),
                        ("DentesCoroa", SPROCKET_TEETH),
                        ("PassoCorrenteMetros", CHAIN_PITCH),
                        ("DiametroPrimitivoCoroaMetros", SPROCKET_PITCH_DIAMETER),
                        ("MassaKg", MASS_KG)):
        log.constant(ROOT + "Telescopio9458/" + name, "double", double(value))

    # Desenho lateral: cada tubo é uma linha vertical com a altura do GLB;
    # a base de cada linha sobe junto com a extensão do estágio.
    key = ROOT + "Telescopio9458/Mecanismo2d"
    mechanism_canvas(log, key, 1.2, 0.3 + 0.8 + 2 * STAGE_TRAVEL)
    stages = []
    for filename, root, color, weight in (("model.glb", "Estagio1Fixo", "#4088ff", 18.0),
                                          ("model_0.glb", "Estagio2Tubo3", "#ff9900", 12.0),
                                          ("model_1.glb", "Estagio3Tubo2", "#44dd88", 6.0)):
        document, _ = read_glb(TELESCOPE / filename)
        lo, hi = Tree(document).box(range(len(document["nodes"])))
        # Altura (Z do robô) das extremidades, com a rotação do config.json.
        z = sorted(robot_frame(config, corner)[2] for corner in (lo, hi))
        bottom = 0.1 + z[0]
        log.constant(key + f"/{root}/x", "double", double(0.6))
        log.start(key + f"/{root}/y", "double")
        mechanism_line(log, key + f"/{root}/Tubo", 90.0, z[1] - z[0], color, weight)
        stages.append((root, bottom))

    plan, duration = telescope_steps()
    for step in range(int(duration / PERIOD) + 1):
        t = step * PERIOD
        micros = round(t * 1e6)
        tube3 = tube2 = telescope_extension(t, plan)  # cascata: andam juntos
        log.tick(micros)
        log.append(ROOT + "Telescopio9458/Pose", micros, pose3d((0.0, 0.0, 0.0), NO_ROTATION))
        # Poses relativas à base: o tubo 2" soma a extensão do tubo 3".
        log.append(ROOT + "Telescopio9458/Componentes", micros,
                   pose3d((0.0, 0.0, tube3), NO_ROTATION) + pose3d((0.0, 0.0, tube3 + tube2), NO_ROTATION))
        log.append(ROOT + "Telescopio9458/ExtensaoTubo3Metros", micros, double(tube3))
        log.append(ROOT + "Telescopio9458/ExtensaoTubo2Metros", micros, double(tube2))
        log.append(ROOT + "Telescopio9458/ExtensaoMetros", micros, double(tube3 + tube2))
        log.append(ROOT + "Telescopio9458/MotorRotacoes", micros, double(tube3 / METERS_PER_MOTOR_ROTATION))
        log.append(ROOT + "Telescopio9458/AlturaTotalMetros", micros, double(CLOSED_HEIGHT + tube3 + tube2))
        for (root, bottom), offset in zip(stages, (0.0, tube3, tube3 + tube2)):
            log.append(key + f"/{root}/y", micros, double(bottom + offset))
    return log.save(TELESCOPE / "Telescopio_9458-Demo.wpilog"), duration


def main():
    argparse.ArgumentParser(description=__doc__).parse_args()
    robot = robot_log()
    telescope, seconds = telescope_log()
    print(json.dumps({"robo": robot, "telescopio9458": telescope,
                      "telescopio9458Segundos": seconds,
                      "telescopio9458CursoPorEstagioMetros": round(STAGE_TRAVEL, 4),
                      "telescopio9458AlturaAbertaMetros": round(CLOSED_HEIGHT + 2 * STAGE_TRAVEL, 4)},
                     ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
