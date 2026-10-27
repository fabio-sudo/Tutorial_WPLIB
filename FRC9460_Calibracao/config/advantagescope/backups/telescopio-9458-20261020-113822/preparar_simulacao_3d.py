"""Prepara abas de validação PhotonLib sem substituir as abas do usuário.

Gera um arquivo para File > Import Layout; não modifica o estado do aplicativo.
"""

from copy import deepcopy
from datetime import date
import json
from pathlib import Path
import uuid


DEST = Path(__file__).resolve().parent
SAVED = Path("C:/Users/Admin/AppData/Roaming/AdvantageScope/state-26_0_0.json")
ROOT = "/RealOutputs/"
REAR = "#00ff00"
FRONT = "#ff8c00"
REFERENCE = "#00ffff"
ERROR = "#ff0000"
MODEL = "FRC9460 - Robo 2026"
# Pose3d[] {intake, torre do shooter}; ver MODELO_3D.md e preparar_componentes.py.
COMPONENTS = "Mecanismos/Componentes"
# Telescópio da equipe 9458 (asset próprio); ver preparar_telescopio_9458.py.
TELESCOPE_MODEL = "FRC9458 - Telescopio"
TELESCOPE_COMPONENTS = "Telescopio9458/Componentes"


def source(kind, path, log_type, **options):
    return dict(type=kind, logKey=ROOT + path, logType=log_type,
                visible=True, options=options)


def ghost(path, color, log_type="Pose3d[]", visible=True):
    item = source("ghost", path, log_type, model=MODEL, color=color)
    item["visible"] = visible
    return item


def series(path, color, log_type="Number"):
    return source("stepped" if log_type == "Number" else "stripes",
                  path, log_type, color=color,
                  **({"size": "normal"} if log_type == "Number" else {}))


def camera_pair(suffix, log_type="Number"):
    return [series(f"Vision/{name}/{suffix}", color, log_type)
            for name, color in (("Rear", REAR), ("Front", FRONT))]


def tab(kind, title, controller, height=235, renderer=None):
    return dict(type=kind, title=title, controller=controller,
                controllerUUID=uuid.uuid4().hex, renderer=renderer,
                controlsHeight=height)


def graph(title, left, right=None, discrete=None):
    return tab(1, title, dict(leftSources=left, rightSources=right or [],
        discreteSources=discrete or [], leftLockedRange=None, rightLockedRange=None,
        leftUnitConversion=dict(autoTarget=None, preset=None),
        rightUnitConversion=dict(autoTarget=None, preset=None), leftFilter=0, rightFilter=0))


def main():
    original = json.loads(SAVED.read_text(encoding="utf-8-sig"))
    original["version"] = "26.0.0"
    backup = DEST / f"Layout-anterior-Simulacao3D-{date.today().isoformat()}.json"
    if not backup.exists():
        backup.write_text(json.dumps(original, ensure_ascii=False, indent=2), encoding="utf-8")

    layout = deepcopy(original)
    hub = layout["hubs"][0]
    state = hub["state"]
    field_template = deepcopy(next(item["controller"] for item in state["tabs"]["tabs"]
                                   if item["type"] == 3))
    field_template["game"] = "FRC:2026 Field"

    # Os alvos são filhos do robô estimado, como exige o tipo Vision Target.
    # As linhas mostram quais tags cada câmera detectou. A origem dessas linhas
    # é o robô; os eixos independentes mostram a montagem real de cada câmera.
    # Filhos (Component, Vision Target) vêm logo depois do robô ao qual pertencem.
    # Sem dados de componentes, o AdvantageScope desenha o robô na posição do CAD.
    robot = [source("robot", "Robot/Pose", "Pose2d", model=MODEL),
             source("component", COMPONENTS, "Pose3d[]")]
    objects = deepcopy(robot)
    for name, color in (("Rear", REAR), ("Front", FRONT)):
        objects.append(source("vision", f"Vision/Simulation/Photon/{name}/TagPoses",
                              "Pose3d[]", color=color, size="normal"))
    objects += [
        ghost("Vision/Simulation/ReferencePose", REFERENCE, "Pose2d"),
        ghost("Vision/Simulation/Photon/Rear/EstimatedRobotPoses", REAR),
        ghost("Vision/Simulation/Photon/Front/EstimatedRobotPoses", FRONT),
        source("axes", "Vision/Simulation/Rear/CameraPose", "Pose3d"),
        source("axes", "Vision/Simulation/Front/CameraPose", "Pose3d"),
    ]
    main_field = deepcopy(field_template)
    main_field["sources"] = objects
    # Orbit Robot segue o Drive, permitindo inspecionar as câmeras sem perder
    # o robô ao atravessar a arena. Posições são no referencial do renderizador.
    follow_robot = dict(cameraIndex=-2, orbitFov=50,
                        cameraPosition=[2.5, 1.8, 3.5], cameraTarget=[0, 0.3, 0])
    tabs = [tab(3, "SIM 3D - Fusao", main_field, 260, follow_robot)]

    applied_field = deepcopy(field_template)
    applied_field["sources"] = deepcopy(robot) + [
        ghost("Vision/Simulation/ReferencePose", REFERENCE, "Pose2d"),
        ghost("Vision/Rear/Fusion/AppliedPoses", REAR, "Pose2d[]"),
        ghost("Vision/Front/Fusion/AppliedPoses", FRONT, "Pose2d[]"),
        ghost("Vision/Rear/Fusion/RejectedPoses", ERROR),
        ghost("Vision/Front/Fusion/RejectedPoses", ERROR),
    ]
    tabs.append(tab(3, "SIM 3D - Aplicadas", applied_field, 260, deepcopy(follow_robot)))
    # Só o robô e seus mecanismos, com a câmera próxima (também serve ao log demo).
    mechanisms_field = deepcopy(field_template)
    mechanisms_field["sources"] = deepcopy(robot)
    tabs.append(tab(3, "SIM 3D - Mecanismos", mechanisms_field, 260,
                    dict(follow_robot, cameraPosition=[1.4, 1.0, 1.4], cameraTarget=[0, 0.35, 0])))
    telescope_field = deepcopy(field_template)
    telescope_field["sources"] = [source("robot", "Robot/Pose", "Pose2d", model=TELESCOPE_MODEL),
                                  source("component", TELESCOPE_COMPONENTS, "Pose3d[]")]
    tabs.append(tab(3, "SIM 3D - Telescopio 9458", telescope_field, 260,
                    dict(follow_robot, cameraPosition=[2.2, 1.6, 2.2], cameraTarget=[0, 0.9, 0])))
    tabs += [
        graph("SIM - Fusao", camera_pair("Fusion/AcceptedCount"), discrete=
              camera_pair("Camera/Connected", "Boolean")
              + camera_pair("Fusion/MeasurementAccepted", "Boolean")
              + camera_pair("Fusion/Reject/CamerasDisagree", "Boolean")),
        graph("SIM - Erros", [series("Vision/Simulation/PositionErrorMeters", REFERENCE)]
              + [series(f"Vision/Simulation/Photon/{name}/EstimationErrorMeters", color)
                 for name, color in (("Rear", REAR), ("Front", FRONT))],
              right=[series("Vision/Simulation/RotationErrorDegrees", "#8e7cc3")],
              discrete=[series(f"Vision/Simulation/Photon/{name}/ReferenceAtCaptureAvailable",
                               color, "Boolean")
                        for name, color in (("Rear", REAR), ("Front", FRONT))]),
        graph("SIM - Tempo", camera_pair("Timing/CaptureAgeSeconds")
              + [series("Vision/Rear/Timing/CaptureAgeLimitSeconds", "#858584")],
              discrete=camera_pair("Fusion/Reject/StaleMeasurement", "Boolean")
              + camera_pair("Fusion/Reject/FutureTimestamp", "Boolean")
              + camera_pair("GyroHealthy", "Boolean")),
        tab(4, "SIM - Diagnostico", [ROOT + f"Vision/{name}/{suffix}"
             for name in ("Rear", "Front")
             for suffix in ("Fusion/FusionResult", "Fusion/LastAppliedAlgorithm",
                            "Fusion/AcceptedCount", "Fusion/BatchSize")]
             + [ROOT + "Vision/Simulation/FusionEnabled",
                ROOT + "Vision/Simulation/PositionErrorMeters",
                ROOT + "Vision/Summary/PendingFrames"], height=0),
    ]

    # Reexecutar atualiza somente as abas deste gerador e conserva as demais.
    titles = {item["title"] for item in tabs}
    previous_tabs = [item for item in state["tabs"]["tabs"] if item["title"] not in titles]
    state["tabs"]["tabs"] = previous_tabs[:1] + tabs + previous_tabs[1:]
    state["tabs"]["selected"] = 1
    state["sidebar"]["width"] = 300
    for path in ("Vision/Simulation", "Vision/Simulation/Photon", "Vision/Simulation/Photon/Rear",
                 "Vision/Simulation/Photon/Front", "Vision/Rear/Fusion", "Vision/Front/Fusion"):
        if ROOT + path not in state["sidebar"]["expanded"]:
            state["sidebar"]["expanded"].append(ROOT + path)

    output = DEST / "FRC9460-Simulacao3D.json"
    output.write_text(json.dumps(layout, ensure_ascii=False, indent=2), encoding="utf-8")
    all_tabs = state["tabs"]["tabs"]
    assert len({item["controllerUUID"] for item in all_tabs}) == len(all_tabs)
    assert all(item["type"] != "cameraOverride" for item in main_field["sources"])
    print(f"Layout: {output}")
    print(f"Backup: {backup}")
    print(f"{len(tabs)} abas novas; {len(previous_tabs)} abas anteriores preservadas.")


if __name__ == "__main__":
    main()
