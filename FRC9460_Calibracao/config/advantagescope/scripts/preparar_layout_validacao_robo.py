"""Gera os painéis integrados de visão, movimento, disparo, constância de tiro, pose e replay.

Uso: python config/advantagescope/scripts/preparar_layout_validacao_robo.py
Somente arquivos de layout; não conecta nem comanda o robô.
"""
import json
import sys
import uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "config" / "elastic"))
import preparar_layouts_elastic as e

REAL = "/RealOutputs/"
REPLAY = "/ReplayOutputs/"
FIELD = "FRC:2026 Field"
MODEL = "FRC9460 - Robo 2026"
MEASURED, TARGET, REAR, FRONT, BAD, SIM = (
    "#00bcd4", "#e48b32", "#7e57c2", "#3b875a", "#e53935", "#c0c0c0")


def source(kind, key, log_type="Number", root=REAL, **options):
    return dict(type=kind, logKey=root + key, logType=log_type, visible=True, options=options)


def line(key, color=MEASURED, root=REAL):
    return source("smooth", key, root=root, color=color, size="normal")


def flag(key, color=MEASURED, log_type="Boolean", root=REAL):
    return source("stripes", key, log_type, root=root, color=color)


def tab(kind, title, controller, renderer=None):
    return dict(type=kind, title=title, controller=controller,
                controllerUUID=uuid.uuid5(uuid.NAMESPACE_URL, "frc9460/validacao/" + title).hex,
                renderer=renderer, controlsHeight=0 if kind == 4 else 220)


def graph(title, left=(), right=(), flags=()):
    return tab(1, title, dict(leftSources=list(left), rightSources=list(right),
        discreteSources=list(flags), leftLockedRange=None, rightLockedRange=None,
        leftUnitConversion=dict(autoTarget=None, preset=None),
        rightUnitConversion=dict(autoTarget=None, preset=None), leftFilter=0, rightFilter=0))


def table(title, keys, root=REAL):
    return tab(4, title, [root + key for key in keys])


# Nos campos o AdvantageScope só oferece estas cores (NeonColors); nos gráficos, qualquer uma.
FIELD_COLORS = {MEASURED: "#00ffff", TARGET: "#ff8c00", REAR: "#ff00ff", FRONT: "#00ff00",
                BAD: "#ff0000", SIM: "#ffff00"}

# Câmeras do 3D. cameraIndex: -1 orbita o campo, -2 orbita o robô, 0 em diante são as
# câmeras do config.json do robô (0 = Limelight traseira).
VIEW_ROBOT = dict(cameraIndex=-2, orbitFov=50, cameraPosition=[3, 3, 4], cameraTarget=[0, 0.3, 0])
VIEW_ROBOT_CLOSE = dict(cameraIndex=-2, orbitFov=50, cameraPosition=[1.4, 1.2, 1.4], cameraTarget=[0, 0.3, 0])
VIEW_SHOT = dict(cameraIndex=-2, orbitFov=50, cameraPosition=[6, 4, 6], cameraTarget=[0, 0.8, 0])
VIEW_FIELD = dict(cameraIndex=-1, orbitFov=50, cameraPosition=[0, 6, -12], cameraTarget=[0, 0.5, 0])
VIEW_LIMELIGHT = dict(cameraIndex=0, orbitFov=50, cameraPosition=[0, 6, -12], cameraTarget=[0, 0.5, 0])


# Filhos pertencem à fonte logo acima: depois de um ghost sairiam do fantasma e
# depois de uma trajetória nem seriam desenhados.
ROBOT_CHILDREN = {"vision", "swerveStates", "component", "mechanism"}


def for_field(sources, three_d):
    """Só as opções que cada aba aceita (conferido no AdvantageScope 26): no 2D o
    robot tem bumpers e o ghost color; no 3D o robot tem model e o ghost model e
    color. Os filhos ficam colados no primeiro robot da lista."""
    result = []
    for item in sources:
        options = dict(item["options"])
        if "color" in options:
            options["color"] = FIELD_COLORS.get(options["color"], options["color"])
        if item["type"] == "robot":
            options = dict(model=MODEL) if three_d else dict(bumpers="")
        elif item["type"] == "ghost":
            options = dict(model=MODEL, color=options["color"]) if three_d else dict(color=options["color"])
        result.append({**item, "options": options})
    children = [item for item in result if item["type"] in ROBOT_CHILDREN]
    others = [item for item in result if item["type"] not in ROBOT_CHILDREN]
    return others[:1] + children + others[1:]


def field(title, sources, three_d=False, view=None):
    sources = for_field(sources, three_d)
    if three_d:
        return tab(3, title, dict(sources=sources, game=FIELD), view or VIEW_ROBOT)
    return tab(2, title, dict(sources=sources, field=FIELD, orientation=0, size="large"))


def robot(key="Robot/Pose", root=REAL, ghost=False, color=MEASURED):
    return source("ghost" if ghost else "robot", key, "Pose2d", root=root, model=MODEL, color=color)


def trajectory(key, color):
    return source("trajectory", key, "Translation3d[]", color=color, size="normal")


def lines(key, color, log_type="Pose3d[]"):
    """Linhas do robô até os pontos (tipo "vision" do AdvantageScope)."""
    return source("vision", key, log_type, color=color, size="normal")


def tag_lines(key, color):
    return lines(key, color, "Translation3d[]")


def heatmap(key, log_type="Pose2d", time_range="full"):
    return source("heatmap", key, log_type, timeRange=time_range)


def cone(key, color, log_type="Pose3d[]"):
    """Marcador colorido no 3D. position=front: a ponta do cone fica no ponto."""
    return source("cone", key, log_type, color=color, position="front")


def swerve(key, color):
    """Vetores dos módulos. Precisa vir logo depois do robot (FL, FR, BL, BR)."""
    return source("swerveStates", key, "SwerveModuleState[]", color=color, arrangement="0,1,2,3")


def fuel(key):
    return source("gamePiece", key, "Translation3d[]", variant="Fuel")


def mechanisms():
    """Peças móveis do modelo 3D. Precisa vir logo depois do robot."""
    return source("component", "Mecanismos/Componentes", "Pose3d[]")


def layout(tabs):
    return dict(hubs=[dict(x=0, y=0, width=1440, height=900, state=dict(
        sidebar=dict(width=270, expanded=["/RealOutputs", "/ReplayOutputs", "/RealOutputs/Fuel"]),
        tabs=dict(selected=0, tabs=tabs)))], satellites=[], version="26.0.0")


def build_analysis():
    poses = [robot(), robot("Vision/Rear/MegaTag2/Pose", ghost=True, color=REAR),
             robot("Vision/Front/MegaTag2/Pose", ghost=True, color=FRONT)]
    shot_lines = [trajectory("Fuel/Tiro/Planejado/Trajetoria", "#ff8c00"),
                  trajectory("Fuel/Tiro/Sensores/Trajetoria", "#00ffff"),
                  trajectory("Shooter/Visualizacao/LinhaAteOAlvo", "#00ff00")]
    event = flag("Fuel/Contagem/DisparoAgora", TARGET)
    seq = "Shooter/Sequencia/"
    tabs = [
        field("01 - Campo: pose, visao e ultimo tiro", [*poses, *shot_lines,
            robot("Validation/Drive2m/TargetPose", ghost=True, color=TARGET),
            source("trajectory", "Validation/Drive2m/PlannedPath", "Pose2d[]", color="#ffff00", size="normal")]),
        field("02 - Robo 3D e trajetorias previstas", [robot(),
            source("component", "Mecanismos/Componentes", "Pose3d[]"), *shot_lines,
            source("axes", "Shooter/Visualizacao/Hub", "Pose3d")], True),
        graph("03 - Liberacao do disparo (s / m)",
            [line(seq + "TempoPreparacaoSegundos")], [line("Shooter/Mira/DistanciaMetros", TARGET)],
            [flag(seq + "Estado", TARGET, "String"), flag(seq + "Detalhe", SIM, "String"),
             flag("Shooter/Mira/PoseConfiavel", REAR), flag("Drive/Vision/RecentCorrection", FRONT),
             *[flag(seq + k, TARGET) for k in ("DentroDaCurva", "HubNoCurso", "PivoPronto", "TopPronto", "TowerPronto", "Pronto")],
             flag(seq + "ProntoPorTempo", BAD), flag(seq + "Alimentando", MEASURED), event]),
        graph("04 - Visao: idade da correcao (s / contagem)",
            [line("Drive/Vision/CorrectionAgeSeconds", REAR), line("Drive/Vision/RecentLimitSeconds", BAD)],
            [line(f"Vision/{c}/Fusion/AcceptedCount", color) for c, color in (("Rear", REAR), ("Front", FRONT))],
            [flag("Drive/Vision/LastAcceptedCamera", SIM, "String"),
             flag("Vision/Diagnostico", REAR, "String"), flag("Drive/Vision/RecentCorrection", FRONT), event]),
        graph("05 - Cameras: latencia (ms / tags)",
            [line(f"Vision/{c}/Timing/ReportedLatencyMs", color) for c, color in (("Rear", REAR), ("Front", FRONT))],
            [line(f"Vision/{c}/MegaTag2/TagCount", color) for c, color in (("Rear", REAR), ("Front", FRONT))],
            [flag(f"Vision/{c}/{k}", color, t) for c, color in (("Rear", REAR), ("Front", FRONT))
             for k, t in (("Camera/Connected", "Boolean"), ("HasTarget", "Boolean"), ("Fusion/FusionResult", "String"))]),
        graph("06 - Movimento: pedido x sensores (m/s / rad/s)",
            [line("Drive/Diagnostic/" + k, c) for k, c in (("DesiredVx", TARGET), ("MeasuredVx", MEASURED),
                ("DesiredVy", FRONT), ("MeasuredVy", REAR))],
            [line("Drive/Diagnostic/DesiredOmega", TARGET), line("Drive/Diagnostic/MeasuredOmega", MEASURED)],
            [flag("Drive/XLockActive", BAD), flag("Drive/Heading/GyroHealthy", FRONT), event]),
        graph("07 - Mira: alvo x pivo (graus / m)",
            [line("Shooter/Pivo/AlvoGraus", TARGET), line("Shooter/Pivo/AnguloGraus"),
             line("Shooter/Pivo/ErroGraus", BAD), line("Shooter/Mira/ErroDirecaoGraus", REAR)],
            [line("Shooter/Mira/DistanciaMetros", FRONT)],
            [flag("Shooter/Pivo/ZeroValido", FRONT), flag("Shooter/Pivo/NoAlvo", MEASURED), event]),
        graph("08 - Rodas: alvo x sensores (RPM)",
            [line(k, c) for k, c in (("Shooter/Top/AlvoRpm", TARGET), ("Shooter/Top/RpmEsquerdo", MEASURED),
                ("Shooter/Top/RpmDireito", SIM), ("Shooter/Tower/AlvoRpm", FRONT),
                ("Shooter/Tower/Rpm", REAR), ("Centrifuga/Rpm", BAD))],
            flags=[flag(seq + "Alimentando", MEASURED), event]),
        graph("09 - Tiro: previsoes x anotacao real (m / RPM)",
            [line("Fuel/Tiro/Planejado/DistanciaDoCentroDoHubMetros", TARGET),
             line("Fuel/Tiro/Sensores/DistanciaDoCentroDoHubMetros"), line("Fuel/Contagem/RaioDoHubMetros", FRONT)],
            [line("Fuel/Contagem/QuedaRpmTop", REAR), line("Fuel/Contagem/LimiteQuedaRpm", BAD)],
            [event, flag("Fuel/Tiro/Visao/CorrecaoRecente", FRONT),
             flag("Fuel/Real/UltimaAnotacao", TARGET, "String")]),
        table("10 - Registro do ultimo disparo", ["Fuel/Tiro/Numero", "Fuel/Real/TiroAnotado", "Fuel/Real/UltimaAnotacao",
            "Fuel/Tiro/Visao/UltimaCamera", "Fuel/Tiro/Visao/SegundosDesdeCorrecao", "Fuel/Tiro/Visao/CorrecaoRecente",
            "Fuel/Tiro/Visao/TirosComVisaoRecente", "Fuel/Tiro/Visao/TirosSemVisaoRecente",
            "Fuel/Tiro/Planejado/Acerta", "Fuel/Tiro/Sensores/Acerta", "Fuel/Tiro/Diferenca/DirecaoGraus",
            "Fuel/Tiro/Diferenca/VelocidadeSaidaMps", "Fuel/Real/Acertos", "Fuel/Real/Erros"]),
        field("SIM 11 - Pose fisica e voo da bola", [robot(),
            source("component", "Mecanismos/Componentes", "Pose3d[]"),
            robot("Drive/Simulation/ReferencePose", ghost=True, color=SIM), *shot_lines,
            trajectory("Fuel/Simulacao/UltimoTiro/Trajetoria", "#ff0000")], True),
        graph("SIM 12 - Resultado fisico simulado (m / bolas)",
            [line("Vision/Simulation/PositionErrorMeters", REAR),
             line("Fuel/Simulacao/UltimoTiro/DistanciaDoCentroDoHubMetros", BAD)],
            [line("Fuel/Simulacao/Lancadas", TARGET), line("Fuel/Simulacao/Acertos", FRONT), line("Fuel/Simulacao/Erros", BAD)],
            [flag("Fuel/Simulacao/UltimoTiro/Resultado", TARGET, "String"), event]),
        graph("13 - Chegada no teste de 2 m (metros / graus)",
            [line("Validation/Drive2m/DistanceMeters"), line("Validation/Drive2m/TargetDistanceMeters", TARGET),
             line("Validation/Drive2m/LateralDistanceMeters", REAR), line("Validation/Drive2m/PositionErrorMeters", BAD),
             line("Validation/Drive2m/PositionToleranceMeters", FRONT)],
            [line("Validation/Drive2m/HeadingErrorDegrees", SIM)],
            [flag("Validation/Drive2m/Active", TARGET), flag("Validation/Drive2m/WithinPositionTolerance", FRONT),
             flag("Validation/Drive2m/Interrupted", BAD)]),
        table("14 - Gravacao e diagnostico", ["Diagnostico/Modo", "Diagnostico/Log/Estado", "Diagnostico/Log/Arquivo",
            "Diagnostico/Log/LivreMiB", "Diagnostico/Log/FilaComFalha", "Vision/Diagnostico",
            *[p + "/Diagnostico/Resultado" for p in ("Intake", "Roller", "Centrifuga", "Shooter/Pivo", "Shooter/Top", "Shooter/Tower")]]),
        # Teste de 29/10/2026: a intake fazia barulho parada porque o PID relaxava e
        # retomava por histerese (20 a 40 trocas por segundo). Hoje o modo só depende
        # da posição: RetencaoEmRepouso = só Brake no batente aberto (0 V);
        # CorrecaoPosicaoAtiva = PID segurando, com tensão constante de poucas dezenas
        # de mV. Qualquer alternância entre os dois com o braço parado é defeito.
        # As entradas do motor (tensão, velocidade) ficam fora de /RealOutputs.
        graph("15 - Intake: segurar a posicao (V / voltas)",
            [line("Intake/Inputs/AppliedVolts", MEASURED, "/")],
            [line("Intake/ErroRetencaoVoltas", BAD), line("Intake/Inputs/VelocityRotationsPerSec", REAR, "/")],
            [flag("Intake/Segurando", FRONT), flag("Intake/RetencaoEmRepouso", TARGET),
             flag("Intake/CorrecaoPosicaoAtiva", MEASURED), flag("Intake/Comando", SIM, "String")]),
        graph("16 - Intake: real x modelo (graus)",
            [line("Intake/Comparacao/RealGraus"), line("Intake/Comparacao/SimuladoGraus", TARGET)],
            [line("Intake/Comparacao/DiferencaGraus", BAD)],
            [flag("Intake/Modelo/ComparacaoAtiva", FRONT), flag("Intake/Comparacao/Divergindo", BAD),
             flag("Intake/ZeroValido", FRONT)]),
        # Heading definido à mão (Start + A) errado entorta a mira e gira a pose
        # do MegaTag2 em volta das tags, sem nenhum outro campo perceber. O MegaTag1
        # calcula o heading pelas tags; só vale parado e vendo 2 ou mais tags (NaN = sem leitura).
        graph("17 - Heading: Pigeon x tags MegaTag1 (graus)",
            [line("Vision/Rear/MegaTag1/ErroHeadingGraus", BAD), line("Shooter/Mira/ErroDirecaoGraus", TARGET)],
            [line("Vision/Rear/MegaTag2/TagCount", REAR)],
            [flag("Vision/Rear/MegaTag1/RoboParado", FRONT), flag("Vision/Rear/HasTarget", MEASURED),
             flag("Vision/Diagnostico", SIM, "String")]),
        field("18 - Campo: pose x MegaTag1 (heading)", [robot(),
            robot("Vision/Rear/MegaTag1/Pose", ghost=True, color=BAD),
            robot("Vision/Rear/MegaTag2/Pose", ghost=True, color=REAR)]),
    ]
    return layout(tabs)


def build_replay():
    def pair(key):
        return [line(key, MEASURED), line(key, TARGET, REPLAY)]
    return layout([
        field("REPLAY 01 - Pose original x recalculada", [robot(), robot(root=REPLAY, ghost=True, color=TARGET),
              source("trajectory", "Validation/Drive2m/PlannedPath", "Pose2d[]", color="#ffff00", size="normal")]),
        graph("REPLAY 02 - Movimento pedido (m/s / rad/s)", pair("Drive/Diagnostic/DesiredVx") + pair("Drive/Diagnostic/DesiredVy"),
              pair("Drive/Diagnostic/DesiredOmega")),
        graph("REPLAY 03 - Mira pedida (graus / m)", pair("Shooter/Pivo/AlvoGraus"), pair("Shooter/Mira/DistanciaMetros")),
        graph("REPLAY 04 - RPM pedido", pair("Shooter/Top/AlvoRpm") + pair("Shooter/Tower/AlvoRpm")),
        graph("REPLAY 05 - Decisoes e disparos (contagem)", pair("Fuel/Contagem/DisparosDetectados"),
            flags=[flag(key, color, t, root) for root, color in ((REAL, MEASURED), (REPLAY, TARGET))
                   for key, t in (("Shooter/Sequencia/Estado", "String"), ("Shooter/Sequencia/Alimentando", "Boolean"),
                                  ("Shooter/Mira/PoseConfiavel", "Boolean"), ("Drive/Vision/RecentCorrection", "Boolean"))]),
        graph("REPLAY 06 - Previsao do tiro (m)", pair("Fuel/Tiro/Planejado/DistanciaDoCentroDoHubMetros")),
    ])


def build_elastic():
    # 1152 x 576 px: cabe em uma tela 1366 x 768 com espaço para os menus.
    e.GRID = 96
    b, t, g = e.boolean, e.text, e.graph
    return e.layout(
        e.tab("01 Operacao",
            t("Modo de execucao", "Diagnostico/Modo", "string", 0, 0, 2),
            t("Estado do disparo", "Shooter/Sequencia/Estado", "string", 2, 0, 6),
            t("Hub escolhido", "Shooter/Mira/HubEscolhido", "string", 8, 0, 4),
            *[b(title, key, i * 2, 1, 2) for i, (title, key) in enumerate([
                ("Pose ja corrigida", "Shooter/Mira/PoseConfiavel"), ("Visao recente", "Drive/Vision/RecentCorrection"),
                ("Pivo pronto", "Shooter/Sequencia/PivoPronto"), ("Top pronto", "Shooter/Sequencia/TopPronto"),
                ("Tower pronto", "Shooter/Sequencia/TowerPronto"), ("Alimentando", "Shooter/Sequencia/Alimentando")])],
            t("Motivo / detalhe da sequencia", "Shooter/Sequencia/Detalhe", "string", 0, 2, 12),
            g("Erro da mira (graus)", "Shooter/Mira/ErroDirecaoGraus", 0, 3, 4, 2, e.ORANGE),
            g("Idade da correcao (s)", "Drive/Vision/CorrectionAgeSeconds", 4, 3, 4, 2, e.PURPLE),
            g("Top medido (RPM)", "Shooter/Top/RpmEsquerdo", 8, 3, 4, 2, e.CYAN),
            t("Distancia ao hub (m)", "Shooter/Mira/DistanciaMetros", "double", 0, 5, 3),
            t("Top solicitado (RPM)", "Shooter/Top/AlvoRpm", "double", 3, 5, 3),
            t("Disparos detectados", "Fuel/Contagem/DisparosDetectados", "int", 6, 5, 3),
            b("Liberado por tempo", "Shooter/Sequencia/ProntoPorTempo", 9, 5, 3, good_when_true=False)),
        e.tab("02 Visao",
            t("Diagnostico da visao", "Vision/Diagnostico", "string", 0, 0, 12),
            *[widget for index, camera in enumerate(("Rear", "Front")) for widget in (
                b("Traseira conectada" if index == 0 else "Frontal opcional", f"Vision/{camera}/Camera/Connected", index * 6, 1, 2),
                b("Vendo tags", f"Vision/{camera}/HasTarget", index * 6 + 2, 1, 2),
                t("Quantidade de tags", f"Vision/{camera}/MegaTag2/TagCount", "int", index * 6 + 4, 1, 2),
                t("Resultado da fusao", f"Vision/{camera}/Fusion/FusionResult", "string", index * 6, 2, 6),
                g("Latencia (ms)", f"Vision/{camera}/Timing/ReportedLatencyMs", index * 6, 3, 3, 2, e.PURPLE),
                g("Desvio informado (m)", f"Vision/{camera}/Fusion/XYStdDevMeters", index * 6 + 3, 3, 3, 2, e.CYAN))],
            t("Ultima camera aceita", "Drive/Vision/LastAcceptedCamera", "string", 0, 5, 3),
            t("Idade correcao (s)", "Drive/Vision/CorrectionAgeSeconds", "double", 3, 5, 3),
            t("Limite recente (s)", "Drive/Vision/RecentLimitSeconds", "double", 6, 5, 3),
            b("Correcao recente", "Drive/Vision/RecentCorrection", 9, 5, 3)),
        e.tab("03 Movimento",
            *[widget for i, (axis, unit) in enumerate((("Vx", "m/s"), ("Vy", "m/s"), ("Omega", "rad/s"))) for widget in (
                g(f"{axis} medido ({unit})", f"Drive/Diagnostic/Measured{axis}", i * 4, 0, 4, 3, e.CYAN),
                t(f"{axis} pedido ({unit})", f"Drive/Diagnostic/Desired{axis}", "double", i * 4, 3, 4),
                t(f"{axis} medido ({unit})", f"Drive/Diagnostic/Measured{axis}", "double", i * 4, 4, 4))],
            b("Giroscopio OK", "Drive/Heading/GyroHealthy", 0, 5, 3),
            b("Trava X ativa", "Drive/XLockActive", 3, 5, 3),
            t("Heading campo (graus)", "Drive/Heading/FieldYawDegrees", "double", 6, 5, 3),
            t("Heading bruto (graus)", "Drive/Heading/RawYawDegrees", "double", 9, 5, 3)),
        e.tab("04 Mira e rodas",
            *[widget for i, (name, measured, target, unit) in enumerate((
                ("Pivo", "Shooter/Pivo/AnguloGraus", "Shooter/Pivo/AlvoGraus", "graus"),
                ("Top", "Shooter/Top/RpmEsquerdo", "Shooter/Top/AlvoRpm", "RPM"),
                ("Tower", "Shooter/Tower/Rpm", "Shooter/Tower/AlvoRpm", "RPM"))) for widget in (
                g(f"{name} medido ({unit})", measured, i * 4, 0, 4, 3, e.CYAN),
                t(f"{name} alvo ({unit})", target, "double", i * 4, 3, 4))],
            b("Zero do pivo valido", "Shooter/Pivo/ZeroValido", 0, 4, 3),
            b("Hub dentro do curso", "Shooter/Sequencia/HubNoCurso", 3, 4, 3),
            b("Dentro da curva", "Shooter/Sequencia/DentroDaCurva", 6, 4, 3),
            t("Centrifuga (RPM)", "Centrifuga/Rpm", "double", 9, 4, 3),
            t("Detalhe da sequencia", "Shooter/Sequencia/Detalhe", "string", 0, 5, 12)),
        e.tab("05 Ultimo tiro",
            t("Tiro detectado", "Fuel/Tiro/Numero", "int", 0, 0, 3),
            t("Tiro anotado", "Fuel/Real/TiroAnotado", "int", 3, 0, 3),
            t("Resultado observado pela equipe", "Fuel/Real/UltimaAnotacao", "string", 6, 0, 6),
            b("Previsao alvo: acerta", "Fuel/Tiro/Planejado/Acerta", 0, 1, 3),
            b("Previsao sensores: acerta", "Fuel/Tiro/Sensores/Acerta", 3, 1, 3),
            b("Visao recente no tiro", "Fuel/Tiro/Visao/CorrecaoRecente", 6, 1, 3),
            t("Camera no tiro", "Fuel/Tiro/Visao/UltimaCamera", "string", 9, 1, 3),
            t("Previsao alvo: erro (m)", "Fuel/Tiro/Planejado/DistanciaDoCentroDoHubMetros", "double", 0, 2, 4),
            t("Previsao sensores: erro (m)", "Fuel/Tiro/Sensores/DistanciaDoCentroDoHubMetros", "double", 4, 2, 4),
            t("Correcao antes do tiro (s)", "Fuel/Tiro/Visao/SegundosDesdeCorrecao", "double", 8, 2, 4),
            g("Queda do top: deteccao (RPM)", "Fuel/Contagem/QuedaRpmTop", 0, 3, 6, 2, e.ORANGE),
            t("Acertos anotados", "Fuel/Real/Acertos", "int", 6, 3, 3),
            t("Erros anotados", "Fuel/Real/Erros", "int", 9, 3, 3),
            t("Tiros sem visao recente", "Fuel/Tiro/Visao/TirosSemVisaoRecente", "int", 6, 4, 6),
            t("SIM: resultado da bola", "Fuel/Simulacao/UltimoTiro/Resultado", "string", 0, 5, 6),
            t("SIM: erro da bola (m)", "Fuel/Simulacao/UltimoTiro/DistanciaDoCentroDoHubMetros", "double", 6, 5, 6)),
        e.tab("06 Gravacao e motores",
            t("Estado da gravacao", "Diagnostico/Log/Estado", "string", 0, 0, 6),
            b("Falha da fila de log", "Diagnostico/Log/FilaComFalha", 6, 0, 3, good_when_true=False),
            t("Espaco livre (MiB)", "Diagnostico/Log/LivreMiB", "double", 9, 0, 3),
            t("Arquivo de log", "Diagnostico/Log/Arquivo", "string", 0, 1, 12),
            *[t(name, prefix + "/Diagnostico/Resultado", "string", (i % 3) * 4, 2 + i // 3, 4)
              for i, (name, prefix) in enumerate((("Pivo", "Shooter/Pivo"), ("Top", "Shooter/Top"),
                  ("Tower", "Shooter/Tower"), ("Centrifuga", "Centrifuga"), ("Intake", "Intake"), ("Rolete", "Roller")))],
            e.alerts(0, 4, 12, 2)),
        e.tab("07 Chegada 2 m",
            b("Teste ativo", "Validation/Drive2m/Active", 0, 0, 3),
            b("Dentro da tolerancia XY", "Validation/Drive2m/WithinPositionTolerance", 3, 0, 3),
            b("Interrompido / timeout", "Validation/Drive2m/Interrupted", 6, 0, 3, good_when_true=False),
            t("Distancia alvo (m)", "Validation/Drive2m/TargetDistanceMeters", "double", 9, 0, 3),
            g("Distancia percorrida (m)", "Validation/Drive2m/DistanceMeters", 0, 1, 4, 3, e.CYAN),
            g("Erro ate o ponto XY (m)", "Validation/Drive2m/PositionErrorMeters", 4, 1, 4, 3, e.RED),
            g("Desvio lateral (m)", "Validation/Drive2m/LateralDistanceMeters", 8, 1, 4, 3, e.PURPLE),
            t("Erro de heading (graus)", "Validation/Drive2m/HeadingErrorDegrees", "double", 0, 4, 4),
            t("Tolerancia XY (m)", "Validation/Drive2m/PositionToleranceMeters", "double", 4, 4, 4),
            t("Distancia final (m)", "Validation/Drive2m/FinalDistanceMeters", "double", 8, 4, 4)),
        e.tab("08 Intake e heading",
            t("Comando da intake", "Intake/Comando", "string", 0, 0, 3),
            b("Segurando a posicao", "Intake/Segurando", 3, 0, 2),
            b("So Brake (batente aberto)", "Intake/RetencaoEmRepouso", 5, 0, 2),
            b("PID ativo", "Intake/CorrecaoPosicaoAtiva", 7, 0, 2),
            t("Diagnostico da intake", "Intake/Diagnostico/Resultado", "string", 9, 0, 3),
            g("Tensao no motor (V)", "Intake/Modelo/TensaoVolts", 0, 1, 6, 2, e.ORANGE),
            g("Erro da retencao (voltas do motor)", "Intake/ErroRetencaoVoltas", 6, 1, 6, 2, e.RED),
            t("Posicao (voltas)", "Intake/PosicaoVoltas", "double", 0, 3, 3),
            t("Alvo de segurar (voltas)", "Intake/AlvoSegurarVoltas", "double", 3, 3, 3),
            t("Angulo de subida (graus)", "Intake/AnguloSubidaGraus", "double", 6, 3, 3),
            t("Corrente real (A)", "Intake/Comparacao/RealCorrenteAmps", "double", 9, 3, 3),
            t("Erro de heading pelas tags (graus)", "Vision/Rear/MegaTag1/ErroHeadingGraus", "double", 0, 4, 4),
            b("Robo parado", "Vision/Rear/MegaTag1/RoboParado", 4, 4, 2),
            b("Traseira ve tags", "Vision/Rear/HasTarget", 6, 4, 2),
            t("Tags (MegaTag2)", "Vision/Rear/MegaTag2/TagCount", "int", 8, 4, 2),
            t("Heading do robo (graus)", "Vision/Rear/RobotYawDegrees", "double", 10, 4, 2),
            t("Erro da mira (graus)", "Shooter/Mira/ErroDirecaoGraus", "double", 0, 5, 4),
            t("Diagnostico da visao", "Vision/Diagnostico", "string", 4, 5, 8)),
    )


# #region CONSTÂNCIA DE TIRO (VELOCIDADE, MIRA E ACERTO ENTRE OS DISPAROS)

def pair(key, root_a=REAL, root_b=REPLAY, color_a=MEASURED, color_b=TARGET):
    """O mesmo campo no log original (RealOutputs) e recalculado (ReplayOutputs)."""
    return [line(key, color_a, root_a), line(key, color_b, root_b)]


def stat(metric, parts):
    """Linhas de Fuel/Consistencia/<medida>/<parte> (Ultimo, Media, Desvio, Minimo, Maximo)."""
    return [line(f"Fuel/Consistencia/{metric}/{part}", color) for part, color in parts]


def build_consistency():
    """Quanto cada disparo se parece com o anterior: velocidade, mira e erro de queda.

    Viés (média longe do ideal) pede calibração; dispersão (desvio grande) pede
    mecânica, alimentação, bateria ou pose. Veja docs/CONSTANCIA_DE_TIRO.md.
    """
    c = "Fuel/Consistencia/"
    seq = "Shooter/Sequencia/"
    event = flag("Fuel/Contagem/DisparoAgora", TARGET)
    full = [("Ultimo", MEASURED), ("Media", TARGET), ("Minimo", SIM), ("Maximo", SIM)]
    tabs = [
        graph("C1 - Velocidade: RPM pedido x real",
            [line(k, col) for k, col in (("Shooter/Top/AlvoRpm", TARGET), ("Shooter/Top/RpmEsquerdo", MEASURED),
                ("Shooter/Top/RpmDireito", SIM), ("Shooter/Tower/AlvoRpm", FRONT), ("Shooter/Tower/Rpm", REAR))],
            flags=[flag(seq + "Alimentando", MEASURED), event]),
        graph("C2 - Disparo: queda de RPM (RPM) e cadencia (s)",
            [line("Fuel/Contagem/QuedaRpmTop", REAR), line("Fuel/Contagem/LimiteQuedaRpm", BAD)],
            stat("IntervaloSegundos", [("Ultimo", TARGET), ("Media", FRONT)]), [event]),
        graph("C3 - Constancia: velocidade de saida (m/s)",
            stat("VelocidadeSaidaMps", full), [line(c + "VelocidadeSaidaMps/Desvio", BAD)],
            [flag(c + "Diagnostico", TARGET, "String"), event]),
        graph("C4 - Constancia: RPM do top no disparo",
            stat("TopRpmNoDisparo", [("Ultimo", MEASURED), ("Media", TARGET)]) + [line("Shooter/Top/AlvoRpm", FRONT)],
            [line(c + "TopRpmErro/Media", BAD), line(c + "TopRpmErro/Desvio", REAR)], [event]),
        graph("C5 - Constancia: mira no disparo (graus)",
            stat("ErroMiraGraus", full) + [line("Shooter/Mira/ErroAteOAlvoAtualGraus", FRONT)],
            [line(c + "ErroMiraGraus/Desvio", BAD), line("Shooter/Pivo/ErroGraus", REAR)],
            [flag("Shooter/Pivo/NoAlvo", MEASURED), event]),
        graph("C6 - Acerto: onde cai a bola (m do centro do hub)",
            [line("Fuel/Tiro/Planejado/DistanciaDoCentroDoHubMetros", TARGET),
             line("Fuel/Tiro/Sensores/DistanciaDoCentroDoHubMetros", MEASURED),
             line("Fuel/Contagem/RaioDoHubMetros", FRONT), line(c + "ErroQuedaMetros/Media", REAR),
             line("Fuel/Simulacao/UltimoTiro/DistanciaDoCentroDoHubMetros", BAD)],
            [line(c + "ErroQuedaMetros/Desvio", SIM)],
            [flag("Fuel/Tiro/Planejado/Acerta", TARGET), flag("Fuel/Tiro/Sensores/Acerta", MEASURED),
             flag("Fuel/Real/UltimaAnotacao", REAR, "String"), flag("Fuel/Simulacao/UltimoTiro/Resultado", BAD, "String")]),
        graph("C7 - Taxa de acerto (0 a 1) e anotacoes da equipe",
            [line(c + "TaxaAcertoPlanejado", TARGET), line(c + "TaxaAcertoSensores", MEASURED),
             line("Fuel/Real/TaxaAcerto", FRONT)],
            [line("Fuel/Real/Acertos", FRONT), line("Fuel/Real/ErrouCurto", REAR), line("Fuel/Real/ErrouLongo", BAD),
             line("Fuel/Real/ErrouDoLado", SIM)],
            [flag("Fuel/Real/UltimaAnotacao", REAR, "String"), event]),
        graph("C8 - Causas: distancia (m), robo (m/s), visao (s) e bateria (V)",
            [line(c + "DistanciaMetros/Ultimo", TARGET), line(c + "VelocidadeRoboMps/Ultimo", MEASURED),
             line("Fuel/Tiro/Visao/SegundosDesdeCorrecao", REAR)],
            [line("SystemStats/BatteryVoltage", FRONT, "/"), line("Fuel/Tiro/Visao/ErroDaPoseMetros", SIM)],
            [flag("Fuel/Tiro/Visao/CorrecaoRecente", FRONT), event]),
        field("C9 - 3D: onde chegou x onde devia",
            [robot(), mechanisms(), *arrival_sources("Fuel/Tiro/"),
             trajectory("Fuel/Tiro/Planejado/Trajetoria", "#ff8c00"), trajectory("Fuel/Tiro/Sensores/Trajetoria", "#00ffff"),
             trajectory("Fuel/Simulacao/UltimoTiro/Trajetoria", "#ff0000")], True, VIEW_SHOT),
        field("C9B - 2D: de onde atira e onde chega",
            [robot(), heatmap("Fuel/Tiro/Visao/PoseEstimada"), heatmap(c + "Chegadas", "Pose3d[]"),
             trajectory("Fuel/Tiro/BocaDoHub", "#ffff00"), trajectory("Fuel/Tiro/Planejado/Trajetoria", "#ff8c00"),
             trajectory("Fuel/Tiro/Sensores/Trajetoria", "#00ffff"), lines("Fuel/Tiro/Sensores/Chegada", MEASURED)]),
        graph("C10 - Replay: constancia original x recalculada",
            pair(c + "VelocidadeSaidaMps/Media") + pair(c + "ErroMiraGraus/Media") + pair(c + "ErroQuedaMetros/Media"),
            pair(c + "VelocidadeSaidaMps/Desvio") + pair(c + "TaxaAcertoPlanejado") + pair(c + "TaxaAcertoSensores"),
            [flag(c + "Diagnostico", MEASURED, "String"), flag(c + "Diagnostico", TARGET, "String", REPLAY)]),
        table("C11 - Ultimo disparo e estatisticas da janela", [
            c + "Diagnostico", c + "TirosNaJanela", "Fuel/Tiro/Numero", "Fuel/Contagem/DisparosDetectados",
            *[f"{c}{m}/{part}" for m in ("VelocidadeSaidaMps", "TopRpmNoDisparo", "TopRpmErro", "QuedaRpmTop",
                                          "ErroMiraGraus", "ErroQuedaMetros", "DistanciaMetros", "VelocidadeRoboMps",
                                          "IntervaloSegundos") for part in ("Ultimo", "Media", "Desvio")],
            c + "TaxaAcertoPlanejado", c + "TaxaAcertoSensores", "Fuel/Real/TaxaAcerto",
            "Fuel/Real/Acertos", "Fuel/Real/ErrouCurto", "Fuel/Real/ErrouLongo", "Fuel/Real/ErrouDoLado"]),
    ]
    return layout(tabs)


def build_consistency_elastic():
    e.GRID = 96
    b, t, g = e.boolean, e.text, e.graph
    c = "Fuel/Consistencia/"

    def row(y, items):
        """Quatro números lado a lado (3 colunas cada)."""
        return [t(title, key, "double", i * 3, y, 3) for i, (title, key) in enumerate(items)]

    return e.layout(
        e.tab("01 Constancia",
            t("Diagnostico da constancia", c + "Diagnostico", "string", 0, 0, 8),
            t("Tiros na janela", c + "TirosNaJanela", "int", 8, 0, 2),
            t("Disparos detectados", "Fuel/Contagem/DisparosDetectados", "int", 10, 0, 2),
            *row(1, [("Velocidade saida: ultimo (m/s)", c + "VelocidadeSaidaMps/Ultimo"),
                     ("Velocidade saida: media (m/s)", c + "VelocidadeSaidaMps/Media"),
                     ("Velocidade saida: desvio (m/s)", c + "VelocidadeSaidaMps/Desvio"),
                     ("Desvio relativo (0,03 = 3%)", c + "CvVelocidade")]),
            *row(2, [("Top no disparo: media (RPM)", c + "TopRpmNoDisparo/Media"),
                     ("Top: pedido - real (RPM)", c + "TopRpmErro/Media"),
                     ("Queda de RPM: media", c + "QuedaRpmTop/Media"),
                     ("Intervalo entre tiros (s)", c + "IntervaloSegundos/Media")]),
            *row(3, [("Mira: ultimo (graus)", c + "ErroMiraGraus/Ultimo"),
                     ("Mira: media = vies (graus)", c + "ErroMiraGraus/Media"),
                     ("Mira: desvio (graus)", c + "ErroMiraGraus/Desvio"),
                     ("Pivo: erro agora (graus)", "Shooter/Pivo/ErroGraus")]),
            *row(4, [("Erro de queda: ultimo (m)", c + "ErroQuedaMetros/Ultimo"),
                     ("Erro de queda: media (m)", c + "ErroQuedaMetros/Media"),
                     ("Erro de queda: desvio (m)", c + "ErroQuedaMetros/Desvio"),
                     ("Raio da boca do hub (m)", "Fuel/Contagem/RaioDoHubMetros")]),
            *row(5, [("Acerto previsto (planejado)", c + "TaxaAcertoPlanejado"),
                     ("Acerto previsto (sensores)", c + "TaxaAcertoSensores"),
                     ("Acerto anotado pela equipe", "Fuel/Real/TaxaAcerto"),
                     ("Ultimo tiro: distancia (m)", c + "DistanciaMetros/Ultimo")])),
        e.tab("02 Anotacoes e graficos",
            t("Ultima anotacao", "Fuel/Real/UltimaAnotacao", "string", 0, 0, 3),
            t("Tiro anotado", "Fuel/Real/TiroAnotado", "int", 3, 0, 1),
            t("Acertou", "Fuel/Real/Acertos", "int", 4, 0, 2),
            t("Errou curto", "Fuel/Real/ErrouCurto", "int", 6, 0, 2),
            t("Errou longo", "Fuel/Real/ErrouLongo", "int", 8, 0, 2),
            t("Errou do lado", "Fuel/Real/ErrouDoLado", "int", 10, 0, 2),
            g("Top medido (RPM)", "Shooter/Top/RpmEsquerdo", 0, 1, 4, 2, e.CYAN),
            g("Queda de RPM do top", "Fuel/Contagem/QuedaRpmTop", 4, 1, 4, 2, e.RED),
            g("Erro de mira agora (graus)", "Shooter/Mira/ErroAteOAlvoAtualGraus", 8, 1, 4, 2, e.ORANGE),
            g("Velocidade de saida prevista (m/s)", "Fuel/Previsao/VelocidadeSaidaMps", 0, 3, 4, 2, e.PURPLE),
            g("Distancia ao hub (m)", "Shooter/Mira/DistanciaMetros", 4, 3, 4, 2, e.CYAN),
            g("Idade da correcao da visao (s)", "Drive/Vision/CorrectionAgeSeconds", 8, 3, 4, 2, e.PURPLE),
            b("Pose confiavel", "Shooter/Mira/PoseConfiavel", 0, 5, 3),
            b("Visao recente", "Drive/Vision/RecentCorrection", 3, 5, 3),
            b("Previsao: acerta (sensores)", "Fuel/Tiro/Sensores/Acerta", 6, 5, 3),
            t("Estado do disparo", "Shooter/Sequencia/Estado", "string", 9, 5, 3)),
    )

# #endregion


# #region POSE E LIMELIGHT

REJECTIONS = ("Disconnected", "NoEstimate", "NoPose", "NoTags", "InvalidPose", "InvalidDistanceData",
              "InvalidTimestamp", "Rotation", "StaleMeasurement", "FutureTimestamp", "UnknownType",
              "Ambiguity", "Height", "Tilt", "OutsideField", "CamerasDisagree")


def build_pose():
    """Pose estimada e Limelight: a câmera vê, a pose é aceita, o heading confere?

    Procedimento de bancada em docs/POSE_E_LIMELIGHT.md.
    """
    r = "Vision/Rear/"
    palette = (REAR, FRONT, MEASURED, TARGET, BAD, SIM)
    ghosts = [robot(r + "MegaTag2/Pose", ghost=True, color=REAR),
              robot("Vision/Front/MegaTag2/Pose", ghost=True, color=FRONT),
              robot(r + "MegaTag1/Pose", ghost=True, color=BAD),
              robot("Drive/Simulation/ReferencePose", ghost=True, color=SIM)]
    tags = [tag_lines(r + "TagsUsadas", REAR), tag_lines("Vision/Front/TagsUsadas", FRONT)]
    tabs = [
        field("P1 - Campo: estimada x MegaTag2 x MegaTag1", [robot(), *ghosts, *tags]),
        field("P2 - Campo 3D: estimada e fantasmas", [robot(), mechanisms(), *ghosts, *tags], True),
        graph("P3 - Camera: tags, distancia (m) e latencia (ms)",
            [line(r + "MegaTag2/TagCount", REAR), line(r + "MegaTag2/AvgTagDist", TARGET)],
            [line(r + "Timing/ReportedLatencyMs", FRONT)],
            [flag(r + "Camera/Connected", MEASURED), flag(r + "HasTarget", REAR), flag(r + "Fusion/Valid", FRONT),
             flag("Vision/Diagnostico", TARGET, "String")]),
        graph("P4 - Correcao: idade (s) e medicoes aceitas",
            [line("Drive/Vision/CorrectionAgeSeconds", REAR), line("Drive/Vision/RecentLimitSeconds", BAD)],
            [line(r + "Fusion/ReceivedCount", SIM), line(r + "Fusion/QueuedCount", TARGET),
             line(r + "Fusion/AcceptedCount", FRONT), line(r + "Fusion/RejectedCount", BAD)],
            [flag("Drive/Vision/RecentCorrection", FRONT), flag("Shooter/Mira/PoseConfiavel", MEASURED),
             flag("Drive/Vision/LastResult", TARGET, "String"), flag(r + "Fusion/FusionResult", REAR, "String")]),
        graph("P5 - Por que rejeita (cada faixa acesa e um filtro)", flags=[
            flag(r + "Fusion/Reject/" + name, palette[i % len(palette)]) for i, name in enumerate(REJECTIONS)]),
        graph("P6 - Inicializacao pelas tags (graus / m)",
            [line("Vision/AutoInit/ErroHeadingGraus", BAD), line(r + "MegaTag1/ErroHeadingGraus", MEASURED)],
            [line("Vision/AutoInit/ErroPosicaoMetros", TARGET), line("Vision/AutoInit/FramesJuntados", REAR)],
            [flag("Vision/AutoInit/Estado", FRONT, "String"), flag("Vision/AutoInit/AplicadaAgora", MEASURED),
             flag("Vision/AutoInit/HeadingDivergente", BAD), flag("Drive/Vision/InicializacaoResultado", SIM, "String")]),
        graph("P7 - Heading do robo x giro (graus / graus por s)",
            [line(r + "RobotYawDegrees", MEASURED), line(r + "MegaTag1/ErroHeadingGraus", BAD)],
            [line(r + "MegaTag2/YawRateDegPerSec", TARGET), line(r + "MegaTag2/YawRateLimitDegPerSec", FRONT)],
            [flag("Drive/Heading/GyroHealthy", FRONT), flag(r + "MegaTag1/RoboParado", MEASURED)]),
        graph("P8 - Erro e confianca (m)",
            [line(r + "Fusion/PoseErrorMeters", BAD), line(r + "Fusion/XYStdDevMeters", TARGET),
             line("Vision/Simulation/PositionErrorMeters", SIM)],
            [line(r + "MegaTag2/FrameAgeSeconds", REAR)],
            [flag(r + "MegaTag2/OutlierSuspected", BAD), flag(r + "Fusion/NewMeasurement", MEASURED),
             flag(r + "Fusion/MeasurementAccepted", FRONT)]),
        table("P9 - Resumo da visao", ["Vision/Diagnostico", "Vision/AutoInit/Estado", "Vision/AutoInit/Resultado",
            "Drive/Vision/LastResult", "Drive/Vision/InicializacaoResultado", "Drive/Vision/InicializacoesTotal",
            "Drive/Vision/LastAcceptedCamera", "Drive/Vision/CorrectionAgeSeconds", "Shooter/Mira/PoseConfiavel",
            r + "Fusion/FusionResult", r + "Fusion/ReceivedCount", r + "Fusion/AcceptedCount", r + "Fusion/RejectedCount"]),
    ]
    return layout(tabs)


def build_pose_elastic():
    e.GRID = 96
    b, t, g = e.boolean, e.text, e.graph
    r = "Vision/Rear/"
    return e.layout(
        e.tab("01 Pose e correcao",
            t("Diagnostico da visao", "Vision/Diagnostico", "string", 0, 0, 12),
            b("Camera conectada", r + "Camera/Connected", 0, 1, 2),
            b("Vendo tags", r + "HasTarget", 2, 1, 2),
            t("Tags (MegaTag2)", r + "MegaTag2/TagCount", "int", 4, 1, 2),
            t("Distancia media (m)", r + "MegaTag2/AvgTagDist", "double", 6, 1, 2),
            b("Correcao recente", "Drive/Vision/RecentCorrection", 8, 1, 2),
            b("Pose confiavel p/ atirar", "Shooter/Mira/PoseConfiavel", 10, 1, 2),
            t("Resultado da fusao", r + "Fusion/FusionResult", "string", 0, 2, 4),
            t("Ultimo resultado do Drive", "Drive/Vision/LastResult", "string", 4, 2, 4),
            t("Ultima camera aceita", "Drive/Vision/LastAcceptedCamera", "string", 8, 2, 4),
            g("Idade da correcao (s)", "Drive/Vision/CorrectionAgeSeconds", 0, 3, 4, 2, e.PURPLE),
            g("Erro pose x visao (m)", r + "Fusion/PoseErrorMeters", 4, 3, 4, 2, e.RED),
            g("Latencia (ms)", r + "Timing/ReportedLatencyMs", 8, 3, 4, 2, e.CYAN),
            t("Recebidas", r + "Fusion/ReceivedCount", "int", 0, 5, 3),
            t("Aceitas", r + "Fusion/AcceptedCount", "int", 3, 5, 3),
            t("Rejeitadas", r + "Fusion/RejectedCount", "int", 6, 5, 3),
            t("Desvio informado (m)", r + "Fusion/XYStdDevMeters", "double", 9, 5, 3)),
        e.tab("02 Heading e inicializacao",
            t("Inicializacao pelas tags", "Vision/AutoInit/Estado", "string", 0, 0, 4),
            t("Resultado", "Drive/Vision/InicializacaoResultado", "string", 4, 0, 4),
            t("Aplicacoes", "Vision/AutoInit/Aplicacoes", "int", 8, 0, 2),
            b("Heading divergente", "Vision/AutoInit/HeadingDivergente", 10, 0, 2, good_when_true=False),
            t("Erro de heading pelas tags (graus)", "Vision/AutoInit/ErroHeadingGraus", "double", 0, 1, 4),
            t("Erro de posicao pelas tags (m)", "Vision/AutoInit/ErroPosicaoMetros", "double", 4, 1, 4),
            t("Quadros juntados", "Vision/AutoInit/FramesJuntados", "int", 8, 1, 4),
            g("Erro de heading MegaTag1 (graus)", r + "MegaTag1/ErroHeadingGraus", 0, 2, 6, 2, e.RED),
            g("Heading do robo (graus)", r + "RobotYawDegrees", 6, 2, 6, 2, e.CYAN),
            b("Giroscopio OK", "Drive/Heading/GyroHealthy", 0, 4, 3),
            b("Robo parado", r + "MegaTag1/RoboParado", 3, 4, 3),
            t("Giro (graus/s)", r + "MegaTag2/YawRateDegPerSec", "double", 6, 4, 3),
            t("Limite de giro (graus/s)", r + "MegaTag2/YawRateLimitDegPerSec", "double", 9, 4, 3),
            e.alerts(0, 5, 12, 1)),
    )

# #endregion


# #region EXPERIÊNCIA 2D E 3D (PARES DE ABAS POR ASSUNTO)

def build_field_experience():
    """O robô em 2D e 3D, por assunto: partida, mecanismos, câmera, tiro, pose, ciclo da bola e replay.

    Usa o que o AdvantageScope oferece: vetores dos módulos, linhas até as tags
    usadas, mapa de calor, cones de chegada, bolas e a visão pela Limelight
    (câmera declarada em mecanismos/Robot_Robo_9460/config.json).
    """
    ghosts = [robot("Vision/Rear/MegaTag2/Pose", ghost=True, color=REAR),
              robot("Vision/Front/MegaTag2/Pose", ghost=True, color=FRONT),
              robot("Vision/Rear/MegaTag1/Pose", ghost=True, color=BAD)]
    tags = [tag_lines("Vision/Rear/TagsUsadas", REAR), tag_lines("Vision/Front/TagsUsadas", FRONT)]
    modules = [swerve("Drive/Swerve/EstadosMedidos", MEASURED), swerve("Drive/Swerve/EstadosPedidos", TARGET)]
    balls = [fuel("Fuel/Simulacao/BolasNoChao"), fuel("Fuel/Simulacao/BolasNoAr")]
    tiro = "Fuel/Tiro/"
    shot_paths = [trajectory(tiro + "Planejado/Trajetoria", "#ff8c00"), trajectory(tiro + "Sensores/Trajetoria", "#00ffff")]
    tabs = [
        field("2D - Partida: pose, modulos e visao",
              [robot(), *modules, *ghosts, *tags, heatmap("Robot/Pose")]),
        field("3D - Partida: campo inteiro",
              [robot(), mechanisms(), *ghosts, *tags, *balls], True, VIEW_FIELD),
        field("3D - Robo de perto: mecanismos e modulos",
              [robot(), mechanisms(), *modules, trajectory("Shooter/Visualizacao/LinhaDoShooter", "#ff8c00"),
               trajectory("Fuel/Previsao/Trajetoria", "#ffff00")], True, VIEW_ROBOT_CLOSE),
        field("3D - Pelos olhos da Limelight traseira", [robot(), mechanisms(), tags[0]], True, VIEW_LIMELIGHT),
        field("2D - Tiro: de onde atira e onde chega",
              [robot(), heatmap(tiro + "Visao/PoseEstimada"), heatmap("Fuel/Consistencia/Chegadas", "Pose3d[]"),
               trajectory(tiro + "BocaDoHub", "#ffff00"), *shot_paths, lines(tiro + "Sensores/Chegada", MEASURED)]),
        field("3D - Tiro: onde chegou x onde devia",
              [robot(), mechanisms(), *arrival_sources(tiro), trajectory("Fuel/Simulacao/UltimoTiro/Trajetoria", "#ff0000"),
               *shot_paths, *balls], True, VIEW_SHOT),
        field("2D - Pose: estimada x tags", [robot(), *ghosts, *tags]),
        field("3D - Pose: estimada x tags", [robot(), mechanisms(), *ghosts, *tags], True),
        field("3D - Ciclo da bola: coleta e disparo",
              [robot(), mechanisms(), *balls, trajectory("Fuel/Previsao/Trajetoria", "#ffff00"), *shot_paths],
              True, VIEW_FIELD),
        field("2D - Replay: robo x codigo atual", [robot(), robot(root=REPLAY, ghost=True, color=TARGET)]),
        field("3D - Replay: robo x codigo atual",
              [robot(), mechanisms(), robot(root=REPLAY, ghost=True, color=TARGET)], True),
        *calibration_tabs(),
    ]
    return layout(tabs)


def calibration_tabs():
    """Log e replay, tensão de cada mecanismo, real x simulado e calibração de PID.

    Simulado aqui é a física recebendo a tensão MEDIDA no motor (ParallelModel):
    existe também nos logs do robô real. Tensão de todos em Modelo/TensaoVolts,
    igual no real e na simulação; top e tower têm um valor por motor em Inputs.
    """
    root = "/"
    battery = line("SystemStats/BatteryVoltage", BAD, root)
    can = line("SystemStats/CANBus/Utilization", SIM, root)
    mechanisms_list = (("Intake", MEASURED), ("Shooter/Pivo", TARGET), ("Shooter/Top", BAD),
                       ("Shooter/Tower", REAR), ("Centrifuga", FRONT), ("Roller", SIM))
    module_colors = (MEASURED, TARGET, REAR, FRONT)

    def volts(prefix, color=REAR):
        return line(prefix + "/Modelo/TensaoVolts", color)

    def amps(prefix, color=BAD):
        return line(prefix + "/Comparacao/RealCorrenteAmps", color)

    def diverging(prefix):
        return flag(prefix + "/Comparacao/Divergindo", BAD)

    def modules(pattern):
        return [line(pattern.format(i), module_colors[i]) for i in range(4)]

    return [
        graph("LOG - Gravando? Ciclo (ms), bateria (V) e CAN (0-1)",
              [line("LoggedRobot/FullCycleMS", BAD), line("LoggedRobot/UserCodeMS", TARGET),
               line("LoggedRobot/LogPeriodicMS", MEASURED), line("Diagnostico/PeriodoAlvoMs", SIM)],
              [battery, can],
              [flag("Diagnostico/Log/GravandoFisico", FRONT), flag("Diagnostico/Log/FilaComFalha", BAD),
               flag("SystemStats/BrownedOut", BAD, root=root), flag("Diagnostico/Modo", SIM, "String")]),
        table("LOG - Arquivo, estado e espaco livre",
              ["Diagnostico/Modo", "Diagnostico/Log/Estado", "Diagnostico/Log/Arquivo", "Diagnostico/Log/LivreMiB",
               "Diagnostico/Log/GravandoFisico", "Diagnostico/Log/FilaComFalha", "Logger/QueuedCycles"]),
        graph("REPLAY - Gravado x codigo atual: mira (graus) e RPM do top",
              pair("Shooter/Pivo/AlvoGraus"), pair("Shooter/Top/AlvoRpm", color_a=REAR, color_b=BAD)),
        graph("MEC - Tensao de todos (V) e bateria",
              [volts(prefix, color) for prefix, color in mechanisms_list], [battery]),
        graph("MEC - Corrente de todos (A) e bateria",
              [amps(prefix, color) for prefix, color in mechanisms_list], [battery]),
        graph("MEC - Real menos simulado (graus / RPM)",
              [line("Intake/Comparacao/DiferencaGraus", MEASURED), line("Shooter/Pivo/Comparacao/DiferencaGraus", TARGET)],
              [line(prefix + "/Comparacao/DiferencaRpm", color) for prefix, color in mechanisms_list[2:]],
              [diverging(prefix) for prefix, _ in mechanisms_list]),
        graph("PID Intake - real x simulado (graus), tensao e corrente",
              [line("Intake/Comparacao/RealGraus", MEASURED), line("Intake/Comparacao/SimuladoGraus", SIM)],
              [volts("Intake"), amps("Intake")],
              [flag("Intake/CorrecaoPosicaoAtiva", TARGET), flag("Intake/RetencaoEmRepouso", FRONT),
               flag("Intake/Comando", SIM, "String"), diverging("Intake")]),
        graph("PID Intake - segurar: erro (voltas) e tensao",
              [line("Intake/AlvoSegurarVoltas", TARGET), line("Intake/PosicaoVoltas", MEASURED)],
              [line("Intake/ErroRetencaoVoltas", BAD), volts("Intake")],
              [flag("Intake/Segurando", FRONT), flag("Intake/Inputs/OpenLimitActive", BAD, root=root),
               flag("Intake/Inputs/CloseLimitActive", BAD, root=root)]),
        graph("PID Pivo - alvo x real x simulado (graus) e erro",
              [line("Shooter/Pivo/AlvoGraus", TARGET), line("Shooter/Pivo/AnguloGraus", MEASURED),
               line("Shooter/Pivo/Comparacao/SimuladoGraus", SIM)],
              [line("Shooter/Pivo/ErroGraus", BAD), volts("Shooter/Pivo")],
              [flag("Shooter/Pivo/NoAlvo", FRONT), flag("Shooter/Pivo/Modo", SIM, "String"), diverging("Shooter/Pivo")]),
        graph("PID Pivo - perfil (voltas), velocidade (voltas/s) e tensao: kV",
              [line("Shooter/Pivo/Inputs/ProfileReferenceRotations", TARGET, root),
               line("Shooter/Pivo/Inputs/MotorPositionRotations", MEASURED, root)],
              [line("Shooter/Pivo/Inputs/MotorVelocityRotationsPerSec", FRONT, root),
               line("Shooter/Pivo/Inputs/AppliedVolts", REAR, root), amps("Shooter/Pivo")],
              [flag("Shooter/Pivo/Inputs/ForwardLimitActive", BAD, root=root),
               flag("Shooter/Pivo/Inputs/ReverseLimitActive", BAD, root=root)]),
        graph("PID Top - pedido x medido x simulado (RPM), tensao e corrente",
              [line("Shooter/Top/AlvoRpm", TARGET), line("Shooter/Top/RpmEsquerdo", MEASURED),
               line("Shooter/Top/RpmDireito", FRONT), line("Shooter/Top/Comparacao/SimuladoRpm", SIM)],
              [volts("Shooter/Top"), amps("Shooter/Top")],
              [flag("Shooter/Top/NoRpm", FRONT), diverging("Shooter/Top")]),
        graph("PID Tower - pedido x medido x simulado (RPM), tensao e corrente",
              [line("Shooter/Tower/AlvoRpm", TARGET), line("Shooter/Tower/Rpm", MEASURED),
               line("Shooter/Tower/Comparacao/SimuladoRpm", SIM)],
              [volts("Shooter/Tower"), amps("Shooter/Tower")],
              [flag("Shooter/Tower/NoRpm", FRONT), diverging("Shooter/Tower")]),
        graph("PID Centrifuga - pedido x medido x simulado (RPM), tensao e erro",
              [line("Centrifuga/AlvoRpm", TARGET), line("Centrifuga/ReferenciaRpm", FRONT), line("Centrifuga/Rpm", MEASURED),
               line("Centrifuga/Comparacao/SimuladoRpm", SIM)],
              [line("Centrifuga/ErroRpm", BAD), volts("Centrifuga")],
              [flag("Centrifuga/NoRpm", FRONT), flag("Centrifuga/Estado", SIM, "String"), diverging("Centrifuga")]),
        graph("MEC Rolete - medido x simulado (RPM), tensao e corrente",
              [line("Roller/Rpm", MEASURED), line("Roller/Comparacao/SimuladoRpm", SIM)],
              [volts("Roller"), line("Roller/CorrenteAmps", BAD)],
              [flag("Roller/Puxando", FRONT), diverging("Roller")]),
        graph("PID Drive - velocidade das rodas (m/s) e erro",
              [line("Drive/Module0/DriveVelocitySetpoint", SIM), *modules("Drive/Module{}/DriveVelocityMeasured")],
              modules("Drive/Module{}/DriveVelocityError")),
        graph("PID Drive - tensao (V) e corrente (A) das rodas",
              [line(f"Drive/Module{i}/DriveAppliedVolts", module_colors[i], root) for i in range(4)],
              [line(f"Drive/Module{i}/DriveCurrentAmps", module_colors[i], root) for i in range(4)]),
        graph("PID Steer - angulo pedido x medido (rad) e erro",
              [line("Drive/Module0/TurnPositionSetpointRad", SIM), *modules("Drive/Module{}/TurnPositionMeasuredRad")],
              modules("Drive/Module{}/TurnErrorRad")),
        graph("PID Chassi - pedido x medido: vx (m/s) e giro (rad/s)",
              [line("Drive/Diagnostic/DesiredVx", TARGET), line("Drive/Diagnostic/MeasuredVx", MEASURED)],
              [line("Drive/Diagnostic/DesiredOmega", REAR), line("Drive/Diagnostic/MeasuredOmega", FRONT)]),
    ]


def arrival_sources(tiro):
    """Anel da boca do hub, centro (onde devia) e cones de onde chegou, com as linhas do erro."""
    return [
        trajectory(tiro + "BocaDoHub", "#ffff00"),
        cone(tiro + "Devia", FRONT, "Pose3d"),
        cone(tiro + "Planejado/Chegada", TARGET), trajectory(tiro + "Planejado/LinhaDoErro", "#ff8c00"),
        cone(tiro + "Sensores/Chegada", MEASURED), trajectory(tiro + "Sensores/LinhaDoErro", "#00ffff"),
        cone("Fuel/Simulacao/UltimoTiro/Chegada", BAD), trajectory("Fuel/Simulacao/UltimoTiro/LinhaDoErro", "#ff0000"),
        cone("Fuel/Consistencia/Chegadas", REAR), cone("Fuel/Simulacao/ChegadasRecentes", SIM),
    ]

# #endregion


# #region ELASTIC: PAINEL ÚNICO DE ANÁLISE

def field_widget(col, row, w, h):
    """Mapa do campo. Lê Campo/Robot; os outros filhos de Campo/ viram objetos (MegaTag2, Hub)."""
    return e._container("Campo", "Field", col, row, w, h, dict(
        topic="/AdvantageKit/RealOutputs/Campo", period=0.06, field_game="Rebuilt",
        robot_width=0.85, robot_length=0.85, show_other_objects=True, show_trajectories=True,
        field_rotation=0.0, robot_color=e.CYAN, trajectory_color=0xFFFFFFFF, show_robot_outside_widget=True))


def camera_widget(col, row, w, h):
    """Vídeo da Limelight: ela publica o próprio tópico em /CameraPublisher (só no robô real)."""
    return e._container("limelight-rear", "Camera Stream", col, row, w, h, dict(
        topic="/CameraPublisher/limelight-rear", period=0.06, rotation_turns=0))


def raw_text(title, topic, data_type, col, row, w=2):
    """Número fora de RealOutputs (bateria, CAN e outros que o AdvantageKit grava sozinho)."""
    return e._container(title, "Text Display", col, row, w, 1, dict(
        topic=topic, period=0.1, data_type=data_type, show_submit_button=False))


def build_analysis_elastic():
    """Uma tela por pergunta: dá para ligar? onde está? atira? os mecanismos? o robô está bem?"""
    e.GRID = 96
    b, t, g = e.boolean, e.text, e.graph
    r = "Vision/Rear/"
    c = "Fuel/Consistencia/"
    seq = "Shooter/Sequencia/"
    motors = (("Pivo", "Shooter/Pivo"), ("Top", "Shooter/Top"), ("Tower", "Shooter/Tower"),
              ("Centrifuga", "Centrifuga"), ("Intake", "Intake"), ("Rolete", "Roller"))
    return e.layout(
        e.tab("01 Pre-jogo",
            t("Modo", "Diagnostico/Modo", "string", 0, 0, 2),
            t("Gravacao do log", "Diagnostico/Log/Estado", "string", 2, 0, 3),
            b("Falha na fila do log", "Diagnostico/Log/FilaComFalha", 5, 0, 2, good_when_true=False),
            t("Espaco livre (MiB)", "Diagnostico/Log/LivreMiB", "double", 7, 0, 2),
            raw_text("Bateria (V)", "/AdvantageKit/SystemStats/BatteryVoltage", "double", 9, 0, 1),
            raw_text("CAN (0-1)", "/AdvantageKit/SystemStats/CANBus/Utilization", "float", 10, 0, 2),
            b("Camera conectada", r + "Camera/Connected", 0, 1, 2),
            b("Giroscopio OK", "Drive/Heading/GyroHealthy", 2, 1, 2),
            b("Zero do pivo", "Shooter/Pivo/ZeroValido", 4, 1, 2),
            b("Zero da intake", "Intake/ZeroValido", 6, 1, 2),
            b("Pose confiavel", "Shooter/Mira/PoseConfiavel", 8, 1, 2),
            b("Heading divergente", "Vision/AutoInit/HeadingDivergente", 10, 1, 2, good_when_true=False),
            *[t(name, prefix + "/Diagnostico/Resultado", "string", (i % 3) * 4, 2 + i // 3, 4)
              for i, (name, prefix) in enumerate(motors)],
            e.alerts(0, 4, 12, 2)),
        e.tab("02 Campo e camera",
            field_widget(0, 0, 7, 4),
            camera_widget(7, 0, 5, 3),
            t("Diagnostico da visao", "Vision/Diagnostico", "string", 7, 3, 5),
            t("Pose (x, y, graus)", "Campo/Robot", "double[]", 0, 4, 4),
            raw_text("Tags usadas", "/AdvantageKit/Vision/Rear/Inputs/TagIds", "int[]", 4, 4, 3),
            t("Inicializacao pelas tags", "Vision/AutoInit/Estado", "string", 7, 4, 3),
            b("Correcao recente", "Drive/Vision/RecentCorrection", 10, 4, 2),
            t("Distancia ao hub (m)", "Shooter/Mira/DistanciaMetros", "double", 0, 5, 3),
            t("Idade da correcao (s)", "Drive/Vision/CorrectionAgeSeconds", "double", 3, 5, 3),
            t("Erro de heading pelas tags (graus)", r + "MegaTag1/ErroHeadingGraus", "double", 6, 5, 3),
            t("Hub escolhido", "Shooter/Mira/HubEscolhido", "string", 9, 5, 3)),
        e.tab("03 Tiro",
            t("Estado do disparo", seq + "Estado", "string", 0, 0, 5),
            t("Constancia", c + "Diagnostico", "string", 5, 0, 7),
            *[b(title, seq + key, i * 2, 1, 2) for i, (title, key) in enumerate([
                ("Hub no curso", "HubNoCurso"), ("Pivo pronto", "PivoPronto"), ("Top pronto", "TopPronto"),
                ("Tower pronta", "TowerPronto"), ("Pronto", "Pronto"), ("Alimentando", "Alimentando")])],
            t("Motivo / detalhe", seq + "Detalhe", "string", 0, 2, 12),
            g("Top medido (RPM)", "Shooter/Top/RpmEsquerdo", 0, 3, 4, 2, e.CYAN),
            g("Erro de mira agora (graus)", "Shooter/Mira/ErroAteOAlvoAtualGraus", 4, 3, 4, 2, e.ORANGE),
            g("Queda de RPM do top", "Fuel/Contagem/QuedaRpmTop", 8, 3, 4, 2, e.RED),
            t("Velocidade: media (m/s)", c + "VelocidadeSaidaMps/Media", "double", 0, 5, 3),
            t("Velocidade: desvio (m/s)", c + "VelocidadeSaidaMps/Desvio", "double", 3, 5, 3),
            t("Erro de queda: media (m)", c + "ErroQuedaMetros/Media", "double", 6, 5, 3),
            t("Acerto anotado", "Fuel/Real/TaxaAcerto", "double", 9, 5, 3)),
        e.tab("04 Mecanismos",
            t("Intake: comando", "Intake/Comando", "string", 0, 0, 3),
            t("Intake: angulo (graus)", "Intake/AnguloSubidaGraus", "double", 3, 0, 3),
            b("So Brake (batente aberto)", "Intake/RetencaoEmRepouso", 6, 0, 3),
            b("PID segurando", "Intake/CorrecaoPosicaoAtiva", 9, 0, 3),
            g("Tensao da intake (V)", "Intake/Modelo/TensaoVolts", 0, 1, 4, 2, e.ORANGE),
            g("Pivo: angulo (graus)", "Shooter/Pivo/AnguloGraus", 4, 1, 4, 2, e.CYAN),
            g("Centrifuga (RPM)", "Centrifuga/Rpm", 8, 1, 4, 2, e.PURPLE),
            t("Pivo: alvo (graus)", "Shooter/Pivo/AlvoGraus", "double", 0, 3, 3),
            t("Pivo: erro (graus)", "Shooter/Pivo/ErroGraus", "double", 3, 3, 3),
            t("Top: alvo (RPM)", "Shooter/Top/AlvoRpm", "double", 6, 3, 3),
            t("Tower: alvo (RPM)", "Shooter/Tower/AlvoRpm", "double", 9, 3, 3),
            t("Rolete (RPM)", "Roller/Rpm", "double", 0, 4, 3),
            t("Rolete: comando", "Roller/Comando", "string", 3, 4, 3),
            t("Centrifuga: estado", "Centrifuga/Estado", "string", 6, 4, 6),
            t("Telescopio: resultado", "Telescopio/Diagnostico/Resultado", "string", 0, 5, 6),
            t("Telescopio: comando", "Telescopio/Comando", "string", 6, 5, 6)),
        e.tab("05 Saude do robo",
            g("Ciclo completo (ms)", "LoggedRobot/FullCycleMS", 0, 0, 6, 2, e.RED, 30.0, 0),
            g("Codigo do robo (ms)", "LoggedRobot/UserCodeMS", 6, 0, 6, 2, e.ORANGE, 30.0, 0),
            t("Fila do logger (ciclos)", "Logger/QueuedCycles", "int", 0, 2, 3),
            raw_text("Bateria (V)", "/AdvantageKit/SystemStats/BatteryVoltage", "double", 3, 2, 3),
            raw_text("CAN (0-1)", "/AdvantageKit/SystemStats/CANBus/Utilization", "float", 6, 2, 3),
            raw_text("Brownout", "/AdvantageKit/SystemStats/BrownedOut", "boolean", 9, 2, 3),
            e.alerts(0, 3, 12, 3)),
    )

# #endregion


def validate_elastic(data):
    for page in data["tabs"]:
        boxes = page["grid_layout"]["containers"]
        for i, a in enumerate(boxes):
            assert 0 <= a["x"] < a["x"] + a["width"] <= 12 * e.GRID, a["title"]
            assert 0 <= a["y"] < a["y"] + a["height"] <= 6 * e.GRID, a["title"]
            for b in boxes[i + 1:]:
                overlap = (a["x"] < b["x"] + b["width"] and b["x"] < a["x"] + a["width"]
                           and a["y"] < b["y"] + b["height"] and b["y"] < a["y"] + a["height"])
                assert not overlap, (page["name"], a["title"], b["title"])


def main():
    dashboards = {
        "config/elastic/layouts/FRC9460-ValidacaoRobo.json": build_elastic(),
        "config/elastic/layouts/FRC9460-ConsistenciaDeTiro.json": build_consistency_elastic(),
        "config/elastic/layouts/FRC9460-PoseLimelight.json": build_pose_elastic(),
        "config/elastic/layouts/FRC9460-Analise.json": build_analysis_elastic(),
    }
    for data in dashboards.values():
        validate_elastic(data)
    outputs = {
        "config/advantagescope/layouts/FRC9460-ValidacaoRobo.json": build_analysis(),
        "config/advantagescope/layouts/FRC9460-Replay.json": build_replay(),
        "config/advantagescope/layouts/FRC9460-ConsistenciaDeTiro.json": build_consistency(),
        "config/advantagescope/layouts/FRC9460-PoseLimelight.json": build_pose(),
        "config/advantagescope/layouts/FRC9460-Campo2D3D.json": build_field_experience(),
        **dashboards,
    }
    for relative, data in outputs.items():
        path = ROOT / relative
        path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(relative)


if __name__ == "__main__":
    main()
