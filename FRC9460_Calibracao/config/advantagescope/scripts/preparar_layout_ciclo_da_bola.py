"""Gera layouts/FRC9460-CicloDaBola.json: intake, roller, centrífuga e as bolas.

Mostra o caminho da bola: coleta (intake + roller), depósito, centrífuga,
disparo e voo até o hub. A aba de contagem compara o que os SENSORES contaram
(Fuel/Contagem, existe no robô real) com a VERDADE da simulação (Fuel/Simulacao).
BOLA 10 e 11: estado da Limelight em cada tiro e pose estimada x verdadeira.
BOLA 12 a 16: real x modelo em paralelo de cada mecanismo e o diagnóstico.

Uso, na raiz do projeto: python config/advantagescope/scripts/preparar_layout_ciclo_da_bola.py
"""

import json
import uuid
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent  # config/advantagescope
OUTPUT = BASE / "layouts" / "FRC9460-CicloDaBola.json"
ROBOT_MODEL = "FRC9460 - Robo 2026"
FIELD = "FRC:2026 Field"

SENSOR = "#e48b32"     # laranja = contado pelos sensores (robô real e simulação)
TRUTH = "#00bcd4"      # ciano = verdade da simulação
GOOD = "#3b875a"       # verde = acerto / pronto
LIMIT = "#e53935"      # vermelho = limites
NEUTRAL = "#c0c0c0"
MODEL = "#00bcd4"      # ciano = modelo em paralelo (física com a tensão medida)
VISION = "#7e57c2"     # roxo = Limelight
# Linhas (trajectory) só aceitam as cores "neon" do AdvantageScope.
LINE_PREDICTED = "#ffff00"  # Yellow: onde a bola iria agora (contínua)
LINE_LAST_SHOT = "#00ffff"  # Cyan: ONDE CHEGOU (voo do último tiro simulado)
LINE_PLANNED = "#00ff00"    # Green: PLANEJADO (RPM e ângulo alvo)
LINE_SENSORS = "#ff00ff"    # Magenta: PELOS SENSORES (RPM e encoder medidos)


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


def real_x_model(title, prefix, unit, color, extra_discrete=()):
    """Real x modelo em paralelo (frc.robot.util.ParallelModel) de um mecanismo."""
    c = out(prefix + "/Comparacao/")
    return graph(title,
                 [line(c + "Real" + unit, color), line(c + "Simulado" + unit, MODEL)],
                 [line(c + "Diferenca" + unit, NEUTRAL), line(c + "Tolerancia" + unit, LIMIT, "stepped")],
                 [stripes(out(prefix + "/Modelo/ComparacaoAtiva"), MODEL),
                  stripes(c + "Divergindo", LIMIT),
                  stripes(out(prefix + "/Diagnostico/Resultado"), NEUTRAL, "String"),
                  *extra_discrete])


def diagnostics_table(title, prefixes, extra=()):
    """Tabela do pit: execução, motor respondendo, resultado e maior diferença."""
    keys = []
    for prefix in prefixes:
        keys += [out(prefix + "/Diagnostico/" + k) for k in ("Execucao", "MotorRespondendo", "Resultado")]
        keys.append(out(prefix + "/Comparacao/Divergindo"))
    return tab(4, title, [*keys, *[out(k) for k in extra]], height=0)


def field3d(title, sources, camera_position):
    return tab(3, title, dict(sources=sources, game=FIELD), height=260,
               renderer=dict(cameraIndex=-2, orbitFov=50, cameraPosition=list(camera_position),
                             cameraTarget=[0, 0.3, 0]))


# Robô com intake e shooter articulados ("component" logo abaixo do "robot").
robot3d = [obj("robot", out("Robot/Pose"), "Pose2d", model=ROBOT_MODEL),
           obj("component", out("Mecanismos/Componentes"), "Pose3d[]")]
# Bolas como o "Fuel" do campo 2026: as do campo somem quando estas existem.
balls3d = [obj("gamePiece", out("Fuel/Simulacao/BolasNoChao"), "Translation3d[]", variant="Fuel"),
           obj("gamePiece", out("Fuel/Simulacao/BolasNoAr"), "Translation3d[]", variant="Fuel")]
lines3d = [obj("trajectory", out("Fuel/Previsao/Trajetoria"), "Translation3d[]", color=LINE_PREDICTED, size="normal"),
           obj("trajectory", out("Fuel/Tiro/Planejado/Trajetoria"), "Translation3d[]", color=LINE_PLANNED, size="bold"),
           obj("trajectory", out("Fuel/Tiro/Sensores/Trajetoria"), "Translation3d[]", color=LINE_SENSORS, size="normal"),
           obj("trajectory", out("Fuel/Simulacao/UltimoTiro/Trajetoria"), "Translation3d[]",
               color=LINE_LAST_SHOT, size="normal"),
           obj("axes", out("Shooter/Visualizacao/Hub"), "Pose3d")]

count = "Fuel/Contagem/"
sim = "Fuel/Simulacao/"
shot = "Fuel/Tiro/Visao/"

tabs = [
    field3d("BOLA 1 - Campo 3D", [*robot3d, *balls3d, *lines3d], (3.0, 3.0, 4.5)),
    field3d("BOLA 2 - Robo de perto", [*robot3d, *balls3d, *lines3d], (1.4, 1.2, 1.4)),
    graph("BOLA 3 - Contagem (sensores x simulacao)",
          [line(out(count + "ColetasDetectadas"), SENSOR, "stepped"),
           line(out(sim + "Coletadas"), TRUTH, "stepped"),
           line(out(count + "DisparosDetectados"), "#ffb74d", "stepped"),
           line(out(sim + "Lancadas"), "#4dd0e1", "stepped"),
           line(out(sim + "Acertos"), GOOD, "stepped"),
           line(out(sim + "Erros"), LIMIT, "stepped"),
           line(out("Fuel/Real/Acertos"), "#a5d6a7", "stepped"),
           line(out("Fuel/Real/Erros"), "#ef9a9a", "stepped")],
          [line(out(count + "NoDepositoEstimado"), SENSOR, "stepped"),
           line(out(sim + "NoDeposito"), TRUTH, "stepped")],
          [stripes(out(count + "ColetaAgora"), SENSOR),
           stripes(out(count + "DisparoAgora"), "#ffb74d"),
           stripes(out(sim + "UltimoTiro/Resultado"), GOOD, "String")]),
    graph("BOLA 4 - Intake",
          [line(out("Intake/AnguloSubidaGraus"), SENSOR),
           line(out("Intake/PosicaoVoltas"), NEUTRAL),
           line(out("Intake/AlvoSegurarVoltas"), GOOD, "stepped")],
          [line("/Intake/Inputs/AppliedVolts", "#5f4528"),
           line("/Intake/Inputs/CurrentAmps", "#8e7cc3")],
          [stripes(out("Intake/Aberto"), GOOD),
           stripes(out("Intake/Fechado"), NEUTRAL),
           stripes(out("Intake/Segurando"), TRUTH),
           stripes("/Intake/Inputs/OpenLimitActive", LIMIT),
           stripes("/Intake/Inputs/CloseLimitActive", LIMIT),
           stripes(out("Intake/ZeroValido"), TRUTH)]),
    graph("BOLA 5 - Roller e coleta",
          [line(out("Roller/CorrenteAmps"), SENSOR),
           line(out(count + "LimiteCorrenteColetaAmps"), LIMIT, "stepped")],
          [line(out("Roller/Rpm"), NEUTRAL)],
          [stripes(out("Roller/Puxando"), GOOD),
           stripes(out(count + "ColetaAgora"), SENSOR)]),
    graph("BOLA 6 - Centrifuga e disparo",
          [line(out("Centrifuga/Rpm"), "#ba68c8"),
           line(out("Centrifuga/AlvoRpm"), "#ba68c8", "stepped"),
           line(out("Shooter/Top/RpmEsquerdo"), SENSOR),
           line(out("Shooter/Top/AlvoRpm"), SENSOR, "stepped")],
          [line(out(count + "QuedaRpmTop"), NEUTRAL),
           line(out(count + "LimiteQuedaRpm"), LIMIT, "stepped")],
          [stripes(out("Centrifuga/Ligada"), "#ba68c8"),
           stripes(out("Centrifuga/NoRpm"), GOOD),
           stripes(out(count + "DisparoAgora"), SENSOR)]),
    graph("BOLA 7 - Tiros",
          [line(out(sim + "UltimoTiro/DistanciaDoCentroDoHubMetros"), TRUTH, "stepped"),
           line(out("Fuel/Previsao/DistanciaDoCentroDoHubMetros"), "#ffff00"),
           line(out(count + "RaioDoHubMetros"), LIMIT, "stepped")],
          [line(out(sim + "UltimoTiro/TempoDeVooSegundos"), NEUTRAL, "stepped"),
           line(out("Fuel/Previsao/VelocidadeSaidaMps"), SENSOR)],
          [stripes(out(sim + "UltimoTiro/Resultado"), GOOD, "String"),
           stripes(out("Fuel/Previsao/AcertaOHub"), GOOD)]),
    graph("BOLA 8 - Tiro planejado x onde chegou",
          [line(out("Fuel/Tiro/Planejado/DistanciaDoCentroDoHubMetros"), GOOD, "stepped"),
           line(out("Fuel/Tiro/Sensores/DistanciaDoCentroDoHubMetros"), "#ba68c8", "stepped"),
           line(out(sim + "UltimoTiro/DistanciaDoCentroDoHubMetros"), TRUTH, "stepped"),
           line(out(count + "RaioDoHubMetros"), LIMIT, "stepped")],
          [line(out("Fuel/Tiro/Diferenca/VelocidadeSaidaMps"), SENSOR, "stepped"),
           line(out("Fuel/Tiro/Diferenca/DirecaoGraus"), NEUTRAL, "stepped")],
          [stripes(out("Fuel/Tiro/Planejado/Acerta"), GOOD),
           stripes(out("Fuel/Tiro/Sensores/Acerta"), "#ba68c8"),
           stripes(out(sim + "UltimoTiro/Resultado"), TRUTH, "String"),
           stripes(out("Fuel/Real/UltimaAnotacao"), SENSOR, "String"),
           stripes(out("Shooter/Modo/MiraAutomaticaLigada"), NEUTRAL)]),
    tab(4, "BOLA 9 - Entradas", [
        *[f"/Intake/Inputs/{f}" for f in (
            "Connected", "ZeroValid", "PositionRotations", "VelocityRotationsPerSec",
            "AppliedVolts", "CurrentAmps", "OpenLimitActive", "CloseLimitActive")],
        *[f"/Roller/Inputs/{f}" for f in ("Connected", "VelocityRpm", "AppliedVolts", "CurrentAmps")],
        *[f"/Centrifuga/Inputs/{f}" for f in ("Connected", "VelocityRpm", "AppliedVolts", "CurrentAmps")],
    ], height=0),
    # ---------- visão em cada tiro ----------
    graph("BOLA 10 - Visao em cada tiro",
          [line(out(shot + "SegundosDesdeCorrecao"), VISION, "stepped"),
           line(out("Drive/Vision/RecentLimitSeconds"), LIMIT, "stepped"),
           line(out(shot + "ErroDaPoseMetros"), TRUTH, "stepped")],
          [line(out(shot + "TirosComVisaoRecente"), GOOD, "stepped"),
           line(out(shot + "TirosSemVisaoRecente"), LIMIT, "stepped")],
          [stripes(out(shot + "CorrecaoRecente"), GOOD),
           stripes(out(shot + "PoseCorrigidaDesdeReset"), VISION),
           stripes(out(shot + "UltimaCamera"), NEUTRAL, "String"),
           stripes(out("Fuel/Tiro/Planejado/Acerta"), GOOD),
           stripes(out(sim + "UltimoTiro/Resultado"), TRUTH, "String")]),
    tab(2, "BOLA 11 - Pose no tiro: estimada x verdadeira", dict(sources=[
        obj("robot", out("Robot/Pose"), "Pose2d", bumpers=""),
        obj("ghost", out(shot + "PoseEstimada"), "Pose2d", color=VISION),
        obj("ghost", out(shot + "PoseVerdadeira"), "Pose2d", color=TRUTH),
        obj("trajectory", out("Fuel/Tiro/Planejado/Trajetoria"), "Translation3d[]", color=LINE_PLANNED, size="bold"),
        obj("trajectory", out(sim + "UltimoTiro/Trajetoria"), "Translation3d[]", color=LINE_LAST_SHOT, size="normal"),
    ], field=FIELD, orientation=0, size="large"), height=200),
    # ---------- real x modelo em paralelo ----------
    real_x_model("BOLA 12 - Intake: real x modelo", "Intake", "Graus", SENSOR,
                 [stripes(out("Intake/Segurando"), TRUTH)]),
    real_x_model("BOLA 13 - Roller: real x modelo", "Roller", "Rpm", SENSOR,
                 [stripes(out(count + "ColetaAgora"), SENSOR)]),
    graph("BOLA 14 - Roller: corrente real x modelo (cada bola)",
          [line(out("Roller/Comparacao/RealCorrenteAmps"), SENSOR),
           line(out("Roller/Comparacao/SimuladoCorrenteAmps"), MODEL),
           line(out(count + "LimiteCorrenteColetaAmps"), LIMIT, "stepped")],
          [line(out("Roller/Comparacao/DiferencaCorrenteAmps"), NEUTRAL)],
          [stripes(out("Roller/Puxando"), GOOD),
           stripes(out(count + "ColetaAgora"), SENSOR)]),
    real_x_model("BOLA 15 - Centrifuga: real x modelo", "Centrifuga", "Rpm", "#ba68c8",
                 [stripes(out("Centrifuga/Ligada"), "#ba68c8")]),
    real_x_model("BOLA 16 - Top: real x modelo (cada tiro)", "Shooter/Top", "Rpm", SENSOR,
                 [stripes(out(count + "DisparoAgora"), SENSOR)]),
    diagnostics_table("BOLA 17 - Diagnostico dos motores",
                      ["Intake", "Roller", "Centrifuga", "Shooter/Pivo", "Shooter/Top", "Shooter/Tower"],
                      ["Intake/Comando", "Roller/Comando", "Centrifuga/Comando", "Shooter/Comando",
                       "Intake/Comparacao/DiferencaMaximaGraus",
                       "Intake/Comparacao/DiferencaAoPararGraus",
                       "Intake/Comparacao/MovimentosComparados",
                       "Roller/Comparacao/DiferencaMaximaRpm",
                       "Centrifuga/Comparacao/DiferencaMaximaRpm"]),
]

layout = dict(hubs=[dict(x=-8, y=-8, width=1552, height=832, state=dict(
    sidebar=dict(width=300, expanded=["/RealOutputs", "/RealOutputs/Fuel", "/Intake", "/Roller", "/Centrifuga"]),
    tabs=dict(selected=0, tabs=tabs)))], satellites=[], version="26.0.0")
OUTPUT.write_text(json.dumps(layout, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print(json.dumps({"layout": str(OUTPUT), "abas": [t["title"] for t in tabs]}, ensure_ascii=False, indent=2))
