"""Gera layouts/FRC9460-Shooter.json: shooter no robô real e na simulação.

Abas "SHOOTER" usam chaves gravadas nos dois modos.
A aba "SIM SHOOTER" usa chaves que só a simulação grava (Shooter/Simulacao/...).
O shooter depende da pose do Drive (odometria + Limelight): as abas de campo
mostram também a pose da visão para conferir de onde vem a mira.

Uso, na raiz do projeto: python config/advantagescope/scripts/preparar_layout_shooter.py
"""

import json
import uuid
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent  # config/advantagescope
OUTPUT = BASE / "layouts" / "FRC9460-Shooter.json"
ROBOT_MODEL = "FRC9460 - Robo 2026"
FIELD = "FRC:2026 Field"

SHOOTER = "#e48b32"   # laranja = para onde o shooter aponta
TARGET = "#3b875a"    # verde = hub / alvo
LIMIT = "#e53935"     # vermelho = limites e rejeições
VISION = "#7e57c2"    # roxo = Limelight
REFERENCE = "#00bcd4"  # ciano = referência da simulação
NEUTRAL = "#c0c0c0"
# Linhas (trajectory) só aceitam as cores "neon" do AdvantageScope.
LINE_SHOOTER = "#ff8c00"  # Orange
LINE_TARGET = "#00ff00"   # Green


def out(key):
    return "/RealOutputs/" + key


def line(key, color, kind="smooth"):
    return dict(type=kind, logKey=key, logType="Number", visible=True,
                options=dict(color=color, size="normal"))


def stripes(key, color, log_type="Boolean"):
    return dict(type="stripes", logKey=key, logType=log_type, visible=True, options=dict(color=color))


def tab(kind, title, controller, height=230, renderer=None):
    return dict(type=kind, title=title, controller=controller, controllerUUID=uuid.uuid4().hex,
                renderer=renderer, controlsHeight=height)


def graph(title, left, right=(), discrete=()):
    return tab(1, title, dict(
        leftSources=list(left), rightSources=list(right), discreteSources=list(discrete),
        leftLockedRange=None, rightLockedRange=None,
        leftUnitConversion=dict(autoTarget=None, preset=None),
        rightUnitConversion=dict(autoTarget=None, preset=None), leftFilter=0, rightFilter=0))


def obj(kind, key, log_type, **options):
    return dict(type=kind, logKey=key, logType=log_type, visible=True, options=options)


def field3d(title, sources, camera_position, game=FIELD):
    return tab(3, title, dict(sources=sources, game=game), height=260,
               renderer=dict(cameraIndex=-2, orbitFov=50, cameraPosition=list(camera_position),
                             cameraTarget=[0, 0.3, 0]))


# Robô com o shooter girando: a fonte "component" logo abaixo do "robot" é a
# lista de poses das peças móveis (model_0 = intake, model_1 = shooter).
robot3d = [obj("robot", out("Robot/Pose"), "Pose2d", model=ROBOT_MODEL),
           obj("component", out("Mecanismos/Componentes"), "Pose3d[]")]
aim_lines = [
    obj("trajectory", out("Shooter/Visualizacao/LinhaDoShooter"), "Pose3d[]", color=LINE_SHOOTER, size="bold"),
    obj("trajectory", out("Shooter/Visualizacao/LinhaAteOAlvo"), "Pose3d[]", color=LINE_TARGET, size="normal"),
    obj("axes", out("Shooter/Visualizacao/Hub"), "Pose3d"),
]
vision3d = [
    obj("ghost", out("Vision/Rear/MegaTag2/Pose"), "Pose2d", model=ROBOT_MODEL, color=VISION),
    obj("ghost", out("Vision/Simulation/ReferencePose"), "Pose2d", model=ROBOT_MODEL, color=REFERENCE),
]

pivot = "Shooter/Pivo/"
seq = "Shooter/Sequencia/"
mira = "Shooter/Mira/"

tabs = [
    # ---------- robô real e simulação ----------
    field3d("SHOOTER 1 - Campo 3D", [*robot3d, *aim_lines, *vision3d], (2.5, 2.2, 3.5)),
    field3d("SHOOTER 2 - Robo de perto", [*robot3d, *aim_lines], (1.3, 1.1, 1.3)),
    tab(2, "SHOOTER 3 - Campo 2D", dict(sources=[
        obj("robot", out("Robot/Pose"), "Pose2d", bumpers=""),
        obj("trajectory", out("Shooter/Visualizacao/LinhaDoShooter"), "Pose3d[]", color=LINE_SHOOTER, size="bold"),
        obj("trajectory", out("Shooter/Visualizacao/LinhaAteOAlvo"), "Pose3d[]", color=LINE_TARGET, size="normal"),
        obj("ghost", out("Vision/Rear/MegaTag2/Pose"), "Pose2d", color=VISION),
        obj("ghost", out("Vision/Simulation/ReferencePose"), "Pose2d", color=REFERENCE),
    ], field=FIELD, orientation=0, size="large"), height=200),
    tab(10, "SHOOTER 4 - Visto de cima", [
        obj("mechanism", out("Shooter/Visualizacao/Mecanismo2d"), "Mechanism2d")], height=200),
    graph("SHOOTER 5 - Pivo",
          [line(out(pivot + "AnguloGraus"), SHOOTER),
           line(out(pivot + "AlvoGraus"), TARGET, "stepped"),
           line(out(mira + "AnguloNecessarioGraus"), NEUTRAL),
           line(out(pivot + "LimiteGraus"), LIMIT, "stepped"),
           line(out(pivot + "LimiteMenosGraus"), LIMIT, "stepped")],
          [line("/Shooter/Pivo/Inputs/AppliedVolts", "#5f4528"),
           line("/Shooter/Pivo/Inputs/StatorCurrentAmps", "#8e7cc3")],
          [stripes(out(pivot + "Modo"), SHOOTER, "String"),
           stripes(out(pivot + "NoAlvo"), TARGET),
           stripes(out(mira + "HubNoCurso"), TARGET),
           stripes(out(pivot + "ZeroValido"), REFERENCE),
           stripes("/Shooter/Pivo/Inputs/ForwardLimitActive", LIMIT),
           stripes("/Shooter/Pivo/Inputs/ReverseLimitActive", LIMIT)]),
    graph("SHOOTER 6 - Rodas (RPM)",
          [line(out("Shooter/Top/RpmEsquerdo"), SHOOTER),
           line(out("Shooter/Top/RpmDireito"), "#ffb74d"),
           line(out("Shooter/Top/AlvoRpm"), SHOOTER, "stepped"),
           line(out("Shooter/Tower/Rpm"), "#4fc3f7"),
           line(out("Shooter/Tower/AlvoRpm"), "#4fc3f7", "stepped"),
           line(out("Shooter/Centrifuga/Rpm"), "#ba68c8"),
           line(out("Shooter/Centrifuga/AlvoRpm"), "#ba68c8", "stepped")],
          discrete=[stripes(out("Shooter/Top/NoRpm"), SHOOTER),
                    stripes(out("Shooter/Tower/NoRpm"), "#4fc3f7"),
                    stripes(out("Shooter/Centrifuga/NoRpm"), "#ba68c8")]),
    graph("SHOOTER 7 - Sequencia de tiro",
          [line(out(seq + "TempoPreparacaoSegundos"), NEUTRAL)],
          [line(out(mira + "DistanciaMetros"), TARGET)],
          [stripes(out(seq + "Estado"), SHOOTER, "String"),
           stripes(out(mira + "PoseConfiavel"), VISION),
           stripes(out(seq + "PivoPronto"), TARGET),
           stripes(out(seq + "TopPronto"), SHOOTER),
           stripes(out(seq + "TowerPronto"), "#4fc3f7"),
           stripes(out(seq + "HubNoCurso"), TARGET),
           stripes(out(seq + "Pronto"), TARGET),
           stripes(out(seq + "Alimentando"), "#ba68c8"),
           stripes(out(seq + "ProntoPorTempo"), LIMIT)]),
    graph("SHOOTER 8 - Mira e Limelight",
          [line(out(mira + "ErroDirecaoGraus"), SHOOTER),
           line(out(mira + "DistanciaMetros"), TARGET)],
          [line(out("Vision/Rear/Fusion/AcceptedCount"), VISION, "stepped"),
           line(out("Vision/Front/Fusion/AcceptedCount"), "#e48b32", "stepped")],
          [stripes(out(mira + "PoseConfiavel"), VISION),
           stripes(out(mira + "Alianca"), NEUTRAL, "String"),
           stripes(out("Vision/Rear/Fusion/MeasurementAccepted"), VISION)]),
    tab(4, "SHOOTER 9 - Entradas", [
        *[f"/Shooter/Pivo/Inputs/{f}" for f in (
            "Connected", "ZeroValid", "MotorPositionRotations", "MotorVelocityRotationsPerSec",
            "AppliedVolts", "StatorCurrentAmps", "ProfileReferenceRotations",
            "ForwardLimitActive", "ReverseLimitActive")],
        *[f"/Shooter/{m}/Inputs/{f}" for m in ("Top", "Tower", "Centrifuga")
          for f in ("Connected", "VelocityRpm", "AppliedVolts", "CurrentAmps")],
    ], height=0),
    # ---------- só simulação ----------
    graph("SIM SHOOTER - Zero manual",
          [line(out(pivot + "AnguloGraus"), SHOOTER),
           line(out("Shooter/Simulacao/AnguloFisicoGraus"), REFERENCE),
           line(out(pivot + "LimiteGraus"), LIMIT, "stepped"),
           line(out(pivot + "LimiteMenosGraus"), LIMIT, "stepped")],
          [line(out("Shooter/Simulacao/ErroDoZeroManualGraus"), NEUTRAL, "stepped")],
          [stripes(out("Shooter/Simulacao/NoBatenteDaFiacao"), LIMIT)]),
]

layout = dict(hubs=[dict(x=-8, y=-8, width=1552, height=832, state=dict(
    sidebar=dict(width=300, expanded=["/RealOutputs", "/RealOutputs/Shooter", "/Shooter", "/Tuning"]),
    tabs=dict(selected=0, tabs=tabs)))], satellites=[], version="26.0.0")
OUTPUT.write_text(json.dumps(layout, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print(json.dumps({"layout": str(OUTPUT), "abas": [t["title"] for t in tabs]}, ensure_ascii=False, indent=2))
