"""Prepara abas de validação PhotonLib sem substituir as abas do usuário.

Gera um arquivo para File > Import Layout; não modifica o estado do aplicativo.
"""

import argparse
from copy import deepcopy
from datetime import date
import json
from pathlib import Path
import uuid


DEST = Path(__file__).resolve().parent.parent  # config/advantagescope
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
# Pose3d da base do telescópio no campo (onde ele está montado).
TELESCOPE_POSE = "Telescopio9458/Pose"
# Telescópio 9460 (etapa 1); campos publicados pelo log dele.
TELESCOPE_9460_MODEL = "FRC9460 - Telescopio - Etapa 1"
# Abas que este gerador criava antes e foram substituídas pelas abas por objeto.
OBSOLETE_TABS = {"SIM 3D - Mecanismos", "SIM 3D - Telescopio 9458", "Mechanism - Telescopio 9458"}


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


# Orbit Robot segue o Drive, permitindo inspecionar as câmeras sem perder
# o robô ao atravessar a arena. Posições são no referencial do renderizador.
FOLLOW_ROBOT = dict(cameraIndex=-2, orbitFov=50,
                    cameraPosition=[2.5, 1.8, 3.5], cameraTarget=[0, 0.3, 0])


def mechanism_tabs():
    """Duas abas por objeto de mecanismos/: Mechanism (desenho 2D) e "3D" (GLBs).

    As abas 3D usam FRC:Axes (sem campo). Os logs de demonstração ficam na
    pasta de cada objeto em mecanismos/.
    """
    objects_3d = {
        "Robo 9460": [source("robot", "Robot/Pose", "Pose2d", model=MODEL),
                      source("component", COMPONENTS, "Pose3d[]")],
        "Telescopio 9460": [source("robot", "Telescopio/BasePose", "Pose2d", model=TELESCOPE_9460_MODEL),
                            source("component", "Telescopio/Componentes", "Pose3d[]")],
        "Telescopio 9458": [source("robot", TELESCOPE_POSE, "Pose3d", model=TELESCOPE_MODEL),
                            source("component", TELESCOPE_COMPONENTS, "Pose3d[]")],
    }
    mechanisms_2d = {"Robo 9460": "Mecanismos/Mecanismo2d",
                     "Telescopio 9460": "Telescopio/Mecanismo2d",
                     "Telescopio 9458": "Telescopio9458/Mecanismo2d"}
    result = []
    for title, sources in objects_3d.items():
        result.append(tab(10, title, [source("mechanism", mechanisms_2d[title], "Mechanism2d")], 200))
        camera = [1.4, 1.0, 1.4, 0.35] if title.startswith("Robo") else [2.6, 2.0, 2.6, 1.0]
        result.append(tab(3, title + " 3D", dict(sources=sources, game="FRC:Axes"), 260,
                          dict(FOLLOW_ROBOT, cameraPosition=camera[:3], cameraTarget=[0, camera[3], 0])))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--estado", type=Path, default=SAVED,
                        help="layout ou estado do AdvantageScope usado como base")
    args = parser.parse_args()
    original = json.loads(args.estado.read_text(encoding="utf-8-sig"))
    original["version"] = "26.0.0"
    backup = DEST / "backups" / "layouts" / f"Layout-anterior-Simulacao3D-{date.today().isoformat()}.json"
    if not backup.exists():
        backup.parent.mkdir(parents=True, exist_ok=True)
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
    follow_robot = deepcopy(FOLLOW_ROBOT)
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
    object_tabs = mechanism_tabs()
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
    tabs = object_tabs + tabs  # abas dos objetos primeiro
    titles = {item["title"] for item in tabs} | OBSOLETE_TABS
    previous_tabs = [item for item in state["tabs"]["tabs"] if item["title"] not in titles]
    state["tabs"]["tabs"] = previous_tabs[:1] + tabs + previous_tabs[1:]
    state["tabs"]["selected"] = 1
    state["sidebar"]["width"] = 300
    for path in ("Vision/Simulation", "Vision/Simulation/Photon", "Vision/Simulation/Photon/Rear",
                 "Vision/Simulation/Photon/Front", "Vision/Rear/Fusion", "Vision/Front/Fusion"):
        if ROOT + path not in state["sidebar"]["expanded"]:
            state["sidebar"]["expanded"].append(ROOT + path)

    output = DEST / "layouts" / "FRC9460-Simulacao3D.json"
    output.write_text(json.dumps(layout, ensure_ascii=False, indent=2), encoding="utf-8")
    all_tabs = state["tabs"]["tabs"]
    assert len({item["controllerUUID"] for item in all_tabs}) == len(all_tabs)
    assert all(item["type"] != "cameraOverride" for item in main_field["sources"])
    print(f"Layout: {output}")
    print(f"Backup: {backup}")
    print(f"{len(tabs)} abas novas; {len(previous_tabs)} abas anteriores preservadas.")


if __name__ == "__main__":
    main()
