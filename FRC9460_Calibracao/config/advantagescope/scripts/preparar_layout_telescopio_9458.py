"""Gera layouts/Telescopio9458-Simulacao.json: teste da simulação Java do telescópio 9458.

Funciona conectado ao robô real ou ao simulador (as chaves são as mesmas). As abas
"Real x modelo" comparam o telescópio com o modelo físico que roda em paralelo.

Segurar RT abre e segurar LT fecha (controle na porta 0). O painel fica em
Tuning > Telescopio9458. As chaves são as de Telescopio_9458 e
TelescopioVisualizer_9458; o 3D usa o asset "FRC9458 - Telescopio".

Uso, na raiz do projeto: python config/advantagescope/scripts/preparar_layout_telescopio_9458.py
"""

import json
import uuid
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent  # config/advantagescope
OUTPUT = BASE / "layouts" / "Telescopio9458-Simulacao.json"
ROOT = "/RealOutputs/Telescopio9458/"
INPUTS = "/Telescopio9458/Inputs/"
TUBE4, TUBE3, TUBE2 = "#4088ff", "#ff9900", "#44dd88"  # mesmas cores do desenho 2D
GOAL, MEASURED, PROFILE, LIMIT = "#ffcc44", "#33bbff", "#b88aff", "#ff2222"
MODEL = "#ff9900"  # modelo em paralelo (real x simulado)


def line(key, color, size="normal"):
    return dict(type="smooth", logKey=key, logType="Number", visible=True,
                options=dict(color=color, size=size))


def stripes(key, color, log_type="Boolean"):
    return dict(type="stripes", logKey=key, logType=log_type, visible=True, options=dict(color=color))


def tab(kind, title, controller, height=235, renderer=None):
    return dict(type=kind, title=title, controller=controller, controllerUUID=uuid.uuid4().hex,
                renderer=renderer, controlsHeight=height)


def graph(title, left, right=(), discrete=()):
    return tab(1, title, dict(
        leftSources=list(left), rightSources=list(right), discreteSources=list(discrete),
        leftLockedRange=None, rightLockedRange=None,
        leftUnitConversion=dict(autoTarget=None, preset=None),
        rightUnitConversion=dict(autoTarget=None, preset=None), leftFilter=0, rightFilter=0))


tabs = [
    tab(3, "Telescopio 9458 3D - RT abre / LT fecha", dict(sources=[
        dict(type="robot", logKey=ROOT + "Pose", logType="Pose3d", visible=True,
             options=dict(model="FRC9458 - Telescopio")),
        dict(type="component", logKey=ROOT + "Componentes", logType="Pose3d[]", visible=True, options={}),
    ], game="FRC:Axes"), height=260,
        renderer=dict(cameraIndex=-2, orbitFov=50, cameraPosition=[2.6, 2.0, 2.6], cameraTarget=[0, 1.0, 0])),
    # Físico (origem): encoder. Virtual (0,6 m ao lado): física pela tensão medida.
    tab(3, "Fisico x virtual 3D - encoder e previsao", dict(sources=[
        dict(type="robot", logKey=ROOT + "Pose", logType="Pose3d", visible=True,
             options=dict(model="FRC9458 - Telescopio")),
        dict(type="component", logKey=ROOT + "Componentes", logType="Pose3d[]", visible=True, options={}),
        dict(type="robot", logKey=ROOT + "Modelo/Pose", logType="Pose3d", visible=True,
             options=dict(model="FRC9458 - Telescopio")),
        dict(type="component", logKey=ROOT + "Modelo/Componentes", logType="Pose3d[]", visible=True,
             options={}),
    ], game="FRC:Axes"), height=260,
        renderer=dict(cameraIndex=-2, orbitFov=50, cameraPosition=[2.6, 2.0, 3.2], cameraTarget=[0, 1.0, -0.3])),
    tab(4, "Voltas - motor e saida do redutor", [
        ROOT + "Voltas/Motor", ROOT + "Voltas/Reducao", ROOT + "Voltas/SaidaRedutor",
        ROOT + "Modelo/VoltasMotor", ROOT + "Modelo/VoltasSaidaRedutor",
        ROOT + "Voltas/DiferencaSaidaRedutor", INPUTS + "PositionValid", ROOT + "Modelo/Valido",
        ROOT + "Limites/OperacionalVoltasMotor", ROOT + "Limites/OperacionalVoltasSaidaRedutor",
    ], height=0),
    tab(4, "Saida da ponta - fisico e simulado (cm e m)", [
        ROOT + "Comparacao/FisicoCentimetros", ROOT + "Comparacao/SimuladoCentimetros",
        ROOT + "Comparacao/DiferencaCentimetros",
        ROOT + "Comparacao/FisicoMetros", ROOT + "Comparacao/SimuladoMetros",
        ROOT + "Comparacao/DiferencaMetros", ROOT + "Comparacao/Valida",
        ROOT + "Modelo/ComparacaoAtiva",
    ], height=0),
    tab(4, "Limites - margem de seguranca na ponta", [
        ROOT + "Limites/MargemCentimetros", ROOT + "Limites/DistanciaBatenteCentimetros",
        ROOT + "Limites/OperacionalCentimetros", ROOT + "Limites/TalonCentimetros",
        ROOT + "Limites/FisicoCentimetros", INPUTS + "PositionValid",
        ROOT + "Controle/Versao", ROOT + "Controle/VelocidadeMaximaMetrosPorSeg",
        ROOT + "Controle/AceleracaoMaximaMetrosPorSeg2", ROOT + "Controle/AproximacaoMetrosPorSeg",
        ROOT + "Controle/FaixaAproximacaoCentimetros",
    ], height=0),
    graph("Saida da ponta - comparacao e limites (cm)",
          [line(ROOT + "Comparacao/FisicoCentimetros", MEASURED),
           line(ROOT + "Comparacao/SimuladoCentimetros", MODEL),
           line(ROOT + "Limites/OperacionalCentimetros", GOAL),
           line(ROOT + "Limites/TalonCentimetros", PROFILE),
           line(ROOT + "Limites/FisicoCentimetros", LIMIT)],
          [line(ROOT + "Comparacao/DiferencaCentimetros", TUBE2)],
          [stripes(ROOT + "Comparacao/Valida", MEASURED),
           stripes(ROOT + "Modelo/ComparacaoAtiva", TUBE2)]),
    graph("Fisico x virtual - voltas na saida do redutor",
          [line(ROOT + "Voltas/SaidaRedutor", MEASURED),
           line(ROOT + "Modelo/VoltasSaidaRedutor", MODEL)],
          [line(ROOT + "Voltas/DiferencaSaidaRedutor", LIMIT)],
          [stripes(ROOT + "Modelo/ComparacaoAtiva", TUBE2)]),
    graph("Fisico x virtual - posicao e erro",
          [line(INPUTS + "ExtensionMeters", MEASURED), line(ROOT + "Modelo/ExtensaoMetros", MODEL),
           line(ROOT + "MetaMetros", GOAL)],
          [line(ROOT + "Modelo/DiferencaMetros", LIMIT)],
          [stripes(ROOT + "Modelo/ComparacaoAtiva", TUBE2),
           stripes(ROOT + "Diagnostico", LIMIT, "String")]),
    graph("Fisico x virtual - corrente e velocidade",
          [line(INPUTS + "MotorCurrentAmps", "#ff6666"), line(ROOT + "Modelo/CorrenteAmps", MODEL)],
          [line(INPUTS + "VelocityMetersPerSec", MEASURED),
           line(ROOT + "Modelo/VelocidadeMetrosPorSeg", "#ffb74d")]),
    graph("Fisico x virtual - tensao e validade",
          [line(ROOT + "TensaoSolicitadaVolts", GOAL), line(INPUTS + "AppliedVolts", MEASURED),
           line(ROOT + "Modelo/TensaoAplicadaVolts", MODEL)],
          discrete=[stripes(ROOT + "Modelo/Valido", TUBE2),
                    stripes(ROOT + "Modelo/FonteTensao", MODEL, "String")]),
    graph("Diagnostico - encoder e conexao",
          [line(INPUTS + "RotorPositionRotations", MEASURED)],
          [line(INPUTS + "ExtensionMeters", TUBE2)],
          [stripes(INPUTS + "MotorConnected", MEASURED),
           stripes(INPUTS + "PositionValid", TUBE2),
           stripes(INPUTS + "ReferenceLostOnReset", LIMIT),
           stripes(INPUTS + "ControlRequestOK", GOAL),
           stripes(ROOT + "Diagnostico", LIMIT, "String"),
           stripes(INPUTS + "ConfigurationStatus", PROFILE, "String"),
           stripes(INPUTS + "ZeroStatus", MODEL, "String"),
           stripes(INPUTS + "ReadStatus", MEASURED, "String"),
           stripes(INPUTS + "ControlStatus", GOAL, "String"),
           stripes(ROOT + "Referencia/Resultado", TUBE2, "String")]),
    graph("PID - erro e parcelas da tensao",
          [line(ROOT + "PID/FeedbackVolts", MEASURED),
           line(ROOT + "PID/FeedforwardVolts", MODEL)],
          [line(ROOT + "PID/ErroPerfilMetros", LIMIT)],
          [stripes(ROOT + "Ajuste/Resultado", TUBE2, "String")]),
    tab(10, "Telescopio 9458 2D", [dict(type="mechanism", logKey=ROOT + "Mecanismo2d",
                                        logType="Mechanism2d", visible=True, options={})], height=120),
    tab(8, "Controle - RT abre / LT fecha",
        ["Xbox Controller (Blue)", "None", "None", "None", "None", "None"], height=85),
    graph("Posicao - meta, medida e comando",
          [line(ROOT + "MetaMetros", GOAL), line(INPUTS + "ExtensionMeters", MEASURED),
           line(ROOT + "ReferenciaPerfilMetros", PROFILE)],
          discrete=[stripes(ROOT + "Comando", "#ff8c00", "String"),
                    stripes(ROOT + "Estado", MEASURED, "String")]),
    graph("Estagios - cascata",
          [line(ROOT + "ExtensaoTubo3Metros", TUBE3), line(ROOT + "ExtensaoTubo2Metros", TUBE2),
           line(ROOT + "ExtensaoMetros", TUBE4)],
          [line(ROOT + "Voltas/Motor", "#cccccc")]),
    graph("Motor - corrente e protecoes",
          [line(ROOT + "TensaoSolicitadaVolts", GOAL), line(INPUTS + "AppliedVolts", MEASURED)],
          [line(INPUTS + "MotorCurrentAmps", "#ff6666"),
           line(ROOT + "Protecao/LimiteCorrenteEstatorAmps", LIMIT, "bold")],
          [stripes(INPUTS + "StatorCurrentLimitActive", "#ff6666"),
           stripes(ROOT + "Protecao/Travado", LIMIT),
           stripes(ROOT + "Protecao/LimiteTopoAtivo", GOAL)]),
    graph("Velocidade da ponta", [line(INPUTS + "VelocityMetersPerSec", TUBE2)]),
]

layout = dict(hubs=[dict(x=-8, y=-8, width=1552, height=832, state=dict(
    sidebar=dict(width=300, expanded=["/Tuning", "/Tuning/Telescopio9458", "/RealOutputs",
                                      "/RealOutputs/Telescopio9458", "/Telescopio9458",
                                      "/Telescopio9458/Inputs"]),
    tabs=dict(selected=1, tabs=tabs)))], satellites=[], version="26.0.0")
OUTPUT.write_text(json.dumps(layout, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print(json.dumps({"layout": str(OUTPUT), "abas": [t["title"] for t in tabs]}, ensure_ascii=False, indent=2))
