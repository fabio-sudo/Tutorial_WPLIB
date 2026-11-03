"""Gera painéis de teste somente de leitura, com as mesmas chaves no real e no SIM.
Modelo = física alimentada pela tensão medida; não é uma segunda medição.
Uso: python config/advantagescope/scripts/preparar_teste_fisico.py
"""
import json
import uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
NAME = "FRC9460-TesteFisicoVsModelo"
OUT = "/RealOutputs/"
BLUE, GOLD, CYAN, RED = "#2196f3", "#ffc107", "#00bcd4", "#ef5350"


def source(key, color, kind="smooth", dtype="Number"):
    return dict(type=kind, logKey=key, logType=dtype, visible=True,
                options=dict(color=color, size="normal"))


def tab(kind, title, controller, renderer=None):
    return dict(type=kind, title=title, controller=controller,
                controllerUUID=uuid.uuid5(uuid.NAMESPACE_URL, NAME + title).hex,
                renderer=renderer, controlsHeight=230 if kind != 4 else 0)


def graph(title, left, right=(), states=()):
    return tab(1, title, dict(leftSources=left, rightSources=list(right),
        discreteSources=list(states), leftLockedRange=None, rightLockedRange=None,
        leftUnitConversion=dict(autoTarget=None, preset=None),
        rightUnitConversion=dict(autoTarget=None, preset=None), leftFilter=0, rightFilter=0))


def line(key, color=BLUE):
    return source(OUT + key, color)


def state(key, dtype="Boolean"):
    return source(OUT + key, RED, "stripes", dtype)


def compare(title, prefix, unit, target, extra=()):
    c = prefix + "/Comparacao/"
    return graph(title, [line(c + "Real" + unit), line(target, GOLD),
                        line(c + "Simulado" + unit, CYAN), *extra],
                 [line(c + "Diferenca" + unit, RED)],
                 [state(prefix + "/Modelo/ComparacaoAtiva"), state(c + "Divergindo")])


def field(title, component):
    sources = [dict(type="robot", logKey=OUT + "Robot/Pose", logType="Pose2d", visible=True,
                    options=dict(model="FRC9460 - Robo 2026")),
               dict(type="component", logKey=OUT + component, logType="Pose3d[]", visible=True, options={})]
    return tab(3, title, dict(sources=sources, game="FRC:2026 Field"),
        dict(cameraIndex=-2, orbitFov=50, cameraPosition=[1.3, 1.1, 1.3], cameraTarget=[0, 0.3, 0]))


tabs = [
    field("1 - 3D medido pelos encoders", "Mecanismos/Componentes"),
    field("2 - 3D modelo pela tensao medida", "Mecanismos/ComponentesModelo"),
    compare("3 - Pivo: graus", "Shooter/Pivo", "Graus", "Shooter/Pivo/AlvoGraus"),
    compare("4 - Top: RPM", "Shooter/Top", "Rpm", "Shooter/Top/AlvoRpm",
            [line("Shooter/Top/RpmDireito", "#9575cd")]),
    compare("5 - Tower: RPM", "Shooter/Tower", "Rpm", "Shooter/Tower/AlvoRpm"),
    compare("6 - Centrifuga: RPM do motor", "Centrifuga", "Rpm", "Centrifuga/AlvoRpm"),
    graph("7 - Intake: graus", [line("Intake/Comparacao/RealGraus"),
          line("Intake/Comparacao/SimuladoGraus", CYAN)],
          [line("Intake/Comparacao/DiferencaGraus", RED)],
          [state("Intake/Modelo/ComparacaoAtiva"), state("Intake/ZeroValido")]),
    graph("8 - Liberacao do tiro", [line("Shooter/Mira/DistanciaMetros")],
          [line("Shooter/Mira/ErroDirecaoGraus", GOLD)],
          [state("Shooter/Sequencia/Estado", "String"),
           *[state("Shooter/Sequencia/" + k) for k in
             ("PivoPronto", "TopPronto", "TowerPronto", "Alimentando", "ProntoPorTempo")],
           state("Shooter/Mira/PoseConfiavel"), state("Shooter/Mira/HubNoCurso")]),
    graph("9 - Swerve: pedido x medido (m/s)",
          [line("Drive/Diagnostic/DesiredVx", GOLD), line("Drive/Diagnostic/MeasuredVx"),
           line("Drive/Diagnostic/DesiredVy", "#ff9800"), line("Drive/Diagnostic/MeasuredVy", CYAN)],
          states=[state("Drive/Heading/GyroHealthy")]),
    graph("10 - Tempo de ciclo (ms)",
          [line("LoggedRobot/UserCodeMS"), line("LoggedRobot/LogPeriodicMS", GOLD),
           line("LoggedRobot/FullCycleMS", CYAN), line("Diagnostico/PeriodoAlvoMs", RED)],
          [line("Logger/QueuedCycles", "#9575cd")], [state("Diagnostico/Log/FilaComFalha")]),
    graph("11 - Bateria e CAN", [source("/SystemStats/BatteryVoltage", GOLD)],
          [source("/SystemStats/CANBus/Utilization", RED)],
          [source("/SystemStats/BrownedOut", RED, "stripes", "Boolean"),
           source("/DriverStation/DSAttached", BLUE, "stripes", "Boolean")]),
    tab(4, "12 - Gravacao e visao", [OUT + k for k in
        ("Diagnostico/Log/Estado", "Diagnostico/Log/Arquivo", "Diagnostico/Log/LivreMiB",
         "Diagnostico/Log/GravandoFisico", "Drive/Vision/LastResult", "Drive/Vision/LastAcceptedCamera",
         "Drive/Vision/CorrectionAgeSeconds", "Centrifuga/SaidaRpm", "Mecanismos/ModeloValido")]),
]
ascope = dict(hubs=[dict(x=0, y=0, width=1552, height=900, state=dict(
    sidebar=dict(width=260, expanded=["/RealOutputs", "/SystemStats"]),
    tabs=dict(selected=2, tabs=tabs)))], satellites=[], version="26.0.0")

# Elastic: telas de 12 colunas, tópicos apenas de leitura, sem controles de motores.
def widget(title, key, kind, dtype, x, y, w=2, h=1, raw=False):
    topic = "/AdvantageKit" + (key if raw else OUT + key)
    props = dict(topic=topic, period=0.1, data_type=dtype, show_submit_button=False)
    if kind == "Graph":
        props.update(time_displayed=15.0, color=0xFF00BCD4, line_width=2.0)
    return dict(title=title, x=x*128, y=y*128, width=w*128, height=h*128,
                type=kind, properties=props)


def text(title, key, x, y, dtype="double", w=2, raw=False):
    return widget(title, key, "Text Display", dtype, x, y, w, raw=raw)


def chart(title, key, x, y, raw=False):
    return widget(title, key, "Graph", "double", x, y, 4, 2, raw)


def panel(name, widgets):
    return dict(name=name, grid_layout=dict(layouts=[], containers=widgets))


panels = [panel("Tiro - motivo e prontidao", [
    text("Estado do tiro", "Shooter/Sequencia/Estado", 0, 0, "string", 4),
    text("Detalhe", "Shooter/Sequencia/Detalhe", 4, 0, "string", 8),
    *[text(title, "Shooter/Sequencia/"+key, i*2, 1, "boolean")
      for i, (title, key) in enumerate([("Pivo pronto", "PivoPronto"), ("Top pronto", "TopPronto"),
       ("Tower pronta", "TowerPronto"), ("Alimentando", "Alimentando"), ("Pronto", "Pronto"),
       ("Fallback RPM", "ProntoPorTempo")])],
    text("Distancia ao hub (m)", "Shooter/Mira/DistanciaMetros", 0, 2),
    text("Pose confiavel", "Shooter/Mira/PoseConfiavel", 2, 2, "boolean"),
    text("Hub no curso", "Shooter/Mira/HubNoCurso", 4, 2, "boolean"),
    text("Centrifuga: estado", "Centrifuga/Estado", 6, 2, "string", 4),
    text("Saida centrifuga RPM (5:1)", "Centrifuga/SaidaRpm", 10, 2),
])]
for name, prefix, unit, target in [
    ("Pivo - graus", "Shooter/Pivo", "Graus", "Shooter/Pivo/AlvoGraus"),
    ("Top - RPM", "Shooter/Top", "Rpm", "Shooter/Top/AlvoRpm"),
    ("Tower - RPM", "Shooter/Tower", "Rpm", "Shooter/Tower/AlvoRpm"),
    ("Centrifuga - RPM motor", "Centrifuga", "Rpm", "Centrifuga/AlvoRpm"),
    ("Intake - graus", "Intake", "Graus", "Intake/AnguloSubidaGraus")]:
    c = prefix + "/Comparacao/"
    panels.append(panel(name, [
        text("Alvo" if prefix != "Intake" else "Angulo medido", target, 0, 0),
        text("Medido", c + "Real"+unit, 2, 0), text("Modelo", c + "Simulado"+unit, 4, 0),
        text("Real menos modelo", c + "Diferenca"+unit, 6, 0),
        text("Comparacao ativa", prefix+"/Modelo/ComparacaoAtiva", 8, 0, "boolean"),
        text("Diagnostico", prefix+"/Diagnostico/Resultado", 10, 0, "string"),
        chart("Medido ("+unit+")", c+"Real"+unit, 0, 1),
        chart("Modelo pela tensao ("+unit+")", c+"Simulado"+unit, 4, 1),
        chart("Diferenca ("+unit+")", c+"Diferenca"+unit, 8, 1),
        text("Corrente real (A)", c+"RealCorrenteAmps", 0, 3),
        text("Corrente modelo (A)", c+"SimuladoCorrenteAmps", 2, 3),
    ]))
panels.append(panel("Swerve e gravacao", [
    text("Log: estado", "Diagnostico/Log/Estado", 0, 0, "string", 4),
    text("Arquivo", "Diagnostico/Log/Arquivo", 4, 0, "string", 8),
    text("Espaco livre (MiB)", "Diagnostico/Log/LivreMiB", 0, 1),
    text("Fila com falha", "Diagnostico/Log/FilaComFalha", 2, 1, "boolean"),
    # O AdvantageKit grava a utilização do CAN como float, não double.
    text("CAN (0-1)", "/SystemStats/CANBus/Utilization", 4, 1, dtype="float", raw=True),
    text("Bateria (V)", "/SystemStats/BatteryVoltage", 6, 1, raw=True),
    text("Ciclo completo (ms)", "LoggedRobot/FullCycleMS", 8, 1),
    text("Orcamento (ms)", "Diagnostico/PeriodoAlvoMs", 10, 1),
    chart("Vx medido (m/s)", "Drive/Diagnostic/MeasuredVx", 0, 2),
    chart("Vx pedido (m/s)", "Drive/Diagnostic/DesiredVx", 4, 2),
    chart("Tempo de ciclo (ms)", "LoggedRobot/FullCycleMS", 8, 2),
]))
elastic = dict(version=1.0, grid_size=128, tabs=panels)

for path, data in [(ROOT / "config/advantagescope/layouts" / (NAME + ".json"), ascope),
                   (ROOT / "config/elastic/layouts" / (NAME + ".json"), elastic)]:
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(path)

# Garante geometrias sem sobreposição, inclusive em futuras edições.
for p in panels:
    items = p["grid_layout"]["containers"]
    for i, a in enumerate(items):
        for b in items[i+1:]:
            overlap = (a["x"] < b["x"]+b["width"] and b["x"] < a["x"]+a["width"]
                       and a["y"] < b["y"]+b["height"] and b["y"] < a["y"]+a["height"])
            assert not overlap, (p["name"], a["title"], b["title"])
print(f"Validado: AdvantageScope {len(tabs)} abas; Elastic {len(panels)} abas, sem sobreposicoes.")
