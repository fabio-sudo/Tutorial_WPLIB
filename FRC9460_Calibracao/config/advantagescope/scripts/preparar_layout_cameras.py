"""Gera layouts/FRC9460-Cameras.json: teste das câmeras reais (Limelight) e virtuais (PhotonVision).

Abas "CAM" usam chaves gravadas pelo Vision nos dois modos (robô real e simulação).
Abas "SIM CAM" usam chaves que só a simulação grava (Vision/Simulation/...).
Na simulação existem Rear e Front; no robô real só Rear, enquanto
VisionConstants.FRONT_CAMERA_ENABLED = false.

Uso, na raiz do projeto: python config/advantagescope/scripts/preparar_layout_cameras.py
"""

import json
import uuid
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent  # config/advantagescope
OUTPUT = BASE / "layouts" / "FRC9460-Cameras.json"
ROBOT_MODEL = "FRC9460 - Robo 2026"
FIELD = "FRC:2026 Field"
CAMERAS = (("Rear", "#3b875a"), ("Front", "#e48b32"))  # verde = traseira, laranja = frontal
REFERENCE = "#00bcd4"  # pose de referência da simulação
REJECTED = "#e53935"
REJECTIONS = ("Disconnected", "NoEstimate", "NoPose", "NoTags", "InvalidPose",
              "InvalidDistanceData", "InvalidTimestamp", "Rotation", "StaleMeasurement",
              "FutureTimestamp", "UnknownType", "Ambiguity", "Height", "Tilt", "OutsideField",
              "CamerasDisagree")


def out(key):
    return "/RealOutputs/" + key


def line(key, color, kind="smooth"):
    return dict(type=kind, logKey=key, logType="Number", visible=True,
                options=dict(color=color, size="normal"))


def stripes(key, color, log_type="Boolean"):
    return dict(type="stripes", logKey=key, logType=log_type, visible=True, options=dict(color=color))


def per_camera(make, suffix, *args, root="Vision/{}/"):
    return [make(out(root.format(name) + suffix), color, *args) for name, color in CAMERAS]


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


def field3d(title, sources, camera_position=(2.5, 1.8, 3.5)):
    return tab(3, title, dict(sources=sources, game=FIELD), height=260,
               renderer=dict(cameraIndex=-2, orbitFov=50, cameraPosition=list(camera_position),
                             cameraTarget=[0, 0.3, 0]))


robot3d = obj("robot", out("Robot/Pose"), "Pose2d", model=ROBOT_MODEL)
estimates3d = [obj("ghost", out(f"Vision/{n}/MegaTag2/Pose"), "Pose2d", model=ROBOT_MODEL, color=c)
               for n, c in CAMERAS]
rejected3d = [obj("ghost", out(f"Vision/{n}/Fusion/RejectedPoses"), "Pose3d[]",
                  model=ROBOT_MODEL, color=REJECTED) for n, _ in CAMERAS]
sim_tags = [obj("vision", out(f"Vision/Simulation/Photon/{n}/TagPoses"), "Pose3d[]", color=c, size="normal")
            for n, c in CAMERAS]
sim_reference3d = obj("ghost", out("Vision/Simulation/ReferencePose"), "Pose2d",
                      model=ROBOT_MODEL, color=REFERENCE)
sim_axes = [obj("axes", out(f"Vision/Simulation/{n}/CameraPose"), "Pose3d") for n, _ in CAMERAS]


def camera_view(name, color):
    """Primeira pessoa: o AdvantageScope coloca a câmera 3D na pose da câmera virtual."""
    return field3d(f"SIM CAM - Visao da camera {name}", [
        obj("cameraOverride", out(f"Vision/Simulation/{name}/CameraPose"), "Pose3d"),
        robot3d,
        obj("vision", out(f"Vision/Simulation/Photon/{name}/TagPoses"), "Pose3d[]", color=color,
            size="normal"),
    ])


def inputs_table(name):
    fields = ("Connected", "Heartbeat", "HasTarget", "TagId", "TagCount", "TxDegrees", "TyDegrees",
              "TargetAreaPercent", "AvgTagDistanceMeters", "LatencyMilliseconds",
              "CaptureTimestampSeconds", "HasPoseEstimate", "Pose")
    return tab(4, f"CAM - Entradas {name}", [f"/Vision/{name}/Inputs/{f}" for f in fields], height=0)


def rejections(name, color):
    return graph(f"CAM - Rejeicoes {name}",
                 [line(out(f"Vision/{name}/Fusion/RejectedCount"), REJECTED, "stepped"),
                  line(out(f"Vision/{name}/Fusion/AcceptedCount"), color, "stepped")],
                 discrete=[stripes(out(f"Vision/{name}/Fusion/Reject/{r}"), REJECTED) for r in REJECTIONS])


tabs = [
    # ---------- robô real e simulação ----------
    field3d("CAM 1 - Campo 3D", [robot3d, *estimates3d, *rejected3d,
                                 sim_reference3d, *sim_tags, *sim_axes]),
    tab(2, "CAM 2 - Campo 2D", dict(sources=[
        obj("robot", out("Robot/Pose"), "Pose2d", bumpers=""),
        *[obj("ghost", out(f"Vision/{n}/MegaTag2/Pose"), "Pose2d", color=c) for n, c in CAMERAS],
        obj("ghost", out("Vision/Simulation/ReferencePose"), "Pose2d", color=REFERENCE),
        *sim_tags,
        *[obj("arrow", out(f"Vision/Simulation/{n}/CameraPose"), "Pose3d", position="center")
          for n, _ in CAMERAS],
    ], field=FIELD, orientation=0, size="large"), height=200),
    graph("CAM 3 - Conexao e tags",
          per_camera(line, "MegaTag2/TagCount", "stepped"),
          per_camera(line, "Camera/Heartbeat"),
          [*per_camera(stripes, "Camera/Connected"), *per_camera(stripes, "HasTarget")]),
    graph("CAM 4 - Tempo",
          per_camera(line, "Timing/ReportedLatencyMs"),
          [*per_camera(line, "Timing/CaptureAgeSeconds"),
           line(out("Vision/Rear/Timing/CaptureAgeLimitSeconds"), REJECTED)],
          [*per_camera(stripes, "Fusion/Reject/StaleMeasurement"),
           *per_camera(stripes, "Fusion/Reject/FutureTimestamp")]),
    graph("CAM 5 - Fusao",
          [*per_camera(line, "Fusion/AcceptedCount", "stepped"),
           *per_camera(line, "Fusion/RejectedCount", "stepped")],
          discrete=[*per_camera(stripes, "Fusion/MeasurementAccepted"),
                    *per_camera(stripes, "Fusion/FusionResult", "String")]),
    graph("CAM 6 - Qualidade",
          [*per_camera(line, "MegaTag2/AvgTagDist"), *per_camera(line, "Fusion/PoseErrorMeters")],
          [*per_camera(line, "Fusion/XYStdDevMeters"),
           line(out("Vision/Summary/CameraDifferenceMeters"), "#7e57c2"),
           line(out("Vision/Summary/CameraDifferenceLimitMeters"), REJECTED)]),
    *[rejections(n, c) for n, c in CAMERAS],
    *[inputs_table(n) for n, _ in CAMERAS],
    # ---------- só simulação ----------
    *[camera_view(n, c) for n, c in CAMERAS],
    graph("SIM CAM - Erro x referencia",
          [*per_camera(line, "EstimationErrorMeters", root="Vision/Simulation/Photon/{}/"),
           line(out("Vision/Simulation/PositionErrorMeters"), REFERENCE)],
          [*per_camera(line, "RotationErrorDegrees", root="Vision/Simulation/Photon/{}/")],
          per_camera(stripes, "MultiTagSuccess", root="Vision/Simulation/Photon/{}/")),
    graph("SIM CAM - Tags vistas",
          [*per_camera(line, "TagCount", "stepped", root="Vision/Simulation/Photon/{}/"),
           *per_camera(line, "UsedTagCount", "stepped", root="Vision/Simulation/Photon/{}/")],
          discrete=per_camera(stripes, "HasTargets", root="Vision/Simulation/Photon/{}/")),
]

layout = dict(hubs=[dict(x=-8, y=-8, width=1552, height=832, state=dict(
    sidebar=dict(width=300, expanded=["/RealOutputs", "/RealOutputs/Vision", "/Vision"]),
    tabs=dict(selected=0, tabs=tabs)))], satellites=[], version="26.0.0")
OUTPUT.write_text(json.dumps(layout, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print(json.dumps({"layout": str(OUTPUT), "abas": [t["title"] for t in tabs]}, ensure_ascii=False, indent=2))
