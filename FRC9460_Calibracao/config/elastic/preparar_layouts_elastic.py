"""Gera os layouts do Elastic em config/elastic/layouts/.

Cada layout do Elastic acompanha um layout do AdvantageScope:
- FRC9460-Shooter      <-> FRC9460-Shooter.json (mira, visão, real x modelo);
- FRC9460-CicloDaBola  <-> FRC9460-CicloDaBola.json (contagens, tiro, coleta);
- Telescopio9460       <-> Telescopio9460-*.json;
- FRC9460-Diagnostico: todos os motores numa tela só, para o pit.

O Elastic mostra o AGORA (números grandes, verde/vermelho, alertas) durante o
treino; o AdvantageScope mostra o histórico e as curvas sobrepostas. Os dois
leem os mesmos nomes do AdvantageKit:
- saídas:  /AdvantageKit/RealOutputs/<chave>  (no log: /RealOutputs/<chave>)
- entradas: /AdvantageKit/<chave>             (no log: /<chave>)
- alertas da WPILib: /SmartDashboard/Alerts

Os arquivos vão para a pasta deploy: o Robot.java serve essa pasta na porta
5800 e o Elastic baixa direto do robô (ou da simulação) em
File > Download Layout. Também dá para abrir o arquivo em File > Open Layout.

Formato conferido no código do Elastic (tab_grid.dart, widget_container_model.dart).
O CicloDaBolaIntegrationTest confere se cada tópico existe no log e com o tipo certo.

Uso, na raiz do projeto: python config/elastic/preparar_layouts_elastic.py
"""

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OUTPUT_DIR = ROOT / "config" / "elastic" / "layouts"

GRID = 128  # tamanho da célula do Elastic, em pixels

# Cores ARGB (o Elastic grava a cor como inteiro).
GREEN = 0xFF4CAF50
RED = 0xFFF44336
CYAN = 0xFF00BCD4
ORANGE = 0xFFFF9800
PURPLE = 0xFF9C27B0
GRAY = 0xFF757575
BLUE = 0xFF29B6F6


# #region WIDGETS

def out(key):
    return "/AdvantageKit/RealOutputs/" + key


def _container(title, kind, col, row, w, h, properties):
    return dict(title=title, x=col * GRID, y=row * GRID, width=w * GRID, height=h * GRID,
                type=kind, properties=properties)


def text(title, key, data_type, col, row, w=2, h=1):
    return _container(title, "Text Display", col, row, w, h,
                      dict(topic=out(key), period=0.06, data_type=data_type, show_submit_button=False))


def boolean(title, key, col, row, w=1, h=1, good_when_true=True):
    """Verde = bom. Para "Divergindo" e parecidos, good_when_true=False."""
    true_color, false_color = (GREEN, RED) if good_when_true else (RED, GREEN)
    return _container(title, "Boolean Box", col, row, w, h,
                      dict(topic=out(key), period=0.06, data_type="boolean",
                           true_color=true_color, false_color=false_color,
                           true_icon="None", false_icon="None"))


def graph(title, key, col, row, w=3, h=2, color=CYAN, seconds=10.0, low=None, high=None):
    props = dict(topic=out(key), period=0.033, data_type="double",
                 time_displayed=seconds, color=color, line_width=2.0)
    if low is not None:
        props["min_value"] = low
    if high is not None:
        props["max_value"] = high
    return _container(title, "Graph", col, row, w, h, props)


def alerts(col, row, w=5, h=3):
    return _container("Alertas (motores sem resposta, sem ID CAN)", "Alerts", col, row, w, h,
                      dict(topic="/SmartDashboard/Alerts", period=0.06))


def tab(name, *containers):
    return dict(name=name, grid_layout=dict(layouts=[], containers=list(containers)))


def layout(*tabs):
    return {"version": 1.0, "grid_size": GRID, "tabs": list(tabs)}


# #endregion

# #region BLOCOS REPETIDOS

def real_x_model(prefix, name, unit, col, row, color):
    """Coluna de 3 de largura: diferença no tempo + situação do modelo."""
    c = prefix + "/Comparacao/"
    return [
        graph(f"{name}: real - modelo ({unit})", c + "Diferenca" + unit, col, row, 3, 2, color),
        boolean("Divergindo", c + "Divergindo", col, row + 2, good_when_true=False),
        text("Resultado", prefix + "/Diagnostico/Resultado", "string", col + 1, row + 2, 2),
        text(f"Real ({unit})", c + "Real" + unit, "double", col, row + 3, 1),
        text(f"Modelo ({unit})", c + "Simulado" + unit, "double", col + 1, row + 3, 1),
        text(f"Maior dif. ({unit})", c + "DiferencaMaxima" + unit, "double", col + 2, row + 3, 1),
    ]


def motor_status(prefix, name, col, row):
    """Linha de 5: motor respondendo, resultado, execução e comando atual."""
    command = prefix.split("/")[0] + "/Comando"
    return [
        boolean(name, prefix + "/Diagnostico/MotorRespondendo", col, row),
        text("Resultado", prefix + "/Diagnostico/Resultado", "string", col + 1, row, 2),
        text("Execucao", prefix + "/Diagnostico/Execucao", "string", col + 3, row, 2),
        text("Comando", command, "string", col + 5, row, 2),
    ]


# #endregion

# #region LAYOUTS

shooter = layout(
    tab("Shooter",
        boolean("Pose confiavel", "Shooter/Mira/PoseConfiavel", 0, 0),
        boolean("Mira automatica (B)", "Shooter/Modo/MiraAutomaticaLigada", 1, 0),
        text("Sequencia de tiro", "Shooter/Sequencia/Estado", "string", 2, 0, 3),
        text("Pivo", "Shooter/Pivo/Modo", "string", 5, 0),
        text("Distancia (m)", "Shooter/Mira/DistanciaMetros", "double", 7, 0),
        boolean("Hub no curso", "Shooter/Mira/HubNoCurso", 9, 0),
        boolean("Pivo no alvo", "Shooter/Pivo/NoAlvo", 10, 0),
        boolean("Top no RPM", "Shooter/Top/NoRpm", 11, 0),
        boolean("Tower no RPM", "Shooter/Tower/NoRpm", 12, 0),
        boolean("Centrifuga no RPM", "Centrifuga/NoRpm", 13, 0),
        graph("Pivo (graus)", "Shooter/Pivo/AnguloGraus", 0, 1, 4, 2, ORANGE, low=-135, high=135),
        graph("Erro de direcao ate o hub (graus)", "Shooter/Mira/ErroDirecaoGraus", 4, 1, 4, 2, RED),
        graph("Top (RPM)", "Shooter/Top/RpmEsquerdo", 8, 1, 3, 2, ORANGE, low=0),
        graph("Tower (RPM)", "Shooter/Tower/Rpm", 11, 1, 3, 2, BLUE, low=0),
        text("Pivo alvo (graus)", "Shooter/Pivo/AlvoGraus", "double", 0, 3),
        text("Angulo necessario", "Shooter/Mira/AnguloNecessarioGraus", "double", 2, 3),
        boolean("Zero do pivo", "Shooter/Pivo/ZeroValido", 4, 3),
        text("Top alvo (RPM)", "Shooter/Top/AlvoRpm", "double", 8, 3),
        text("Tower alvo (RPM)", "Shooter/Tower/AlvoRpm", "double", 11, 3),
        text("Centrifuga (RPM)", "Centrifuga/Rpm", "double", 5, 3),
        text("Alianca", "Shooter/Mira/Alianca", "string", 0, 4),
        text("Hub escolhido", "Shooter/Mira/HubEscolhido", "string", 2, 4, 3),
    ),
    tab("Visao",
        boolean("Corrigiu desde o reset", "Drive/Vision/CorrectedSinceReset", 0, 0, 2),
        boolean("Correcao recente (1 s)", "Drive/Vision/RecentCorrection", 2, 0, 2),
        text("Ultima camera aceita", "Drive/Vision/LastAcceptedCamera", "string", 4, 0),
        text("Ultimo resultado", "Drive/Vision/LastResult", "string", 6, 0),
        boolean("Frontal ve tag", "Vision/Front/HasTarget", 8, 0),
        boolean("Traseira ve tag", "Vision/Rear/HasTarget", 9, 0),
        text("Aceitas frontal", "Drive/Vision/Front/AcceptedCount", "int", 10, 0),
        text("Aceitas traseira", "Drive/Vision/Rear/AcceptedCount", "int", 12, 0),
        graph("Idade da ultima correcao (s)", "Drive/Vision/CorrectionAgeSeconds", 0, 1, 5, 2, PURPLE, 20.0, 0, 3),
        graph("Erro da pose na simulacao (m)", "Vision/Simulation/PositionErrorMeters", 5, 1, 5, 2, CYAN, 20.0, 0),
        text("Fusao frontal", "Vision/Front/Fusion/FusionResult", "string", 10, 1, 4),
        text("Fusao traseira", "Vision/Rear/Fusion/FusionResult", "string", 10, 2, 4),
        boolean("Pose confiavel p/ atirar", "Shooter/Mira/PoseConfiavel", 0, 3, 2),
        text("Por que a visao nao corrige", "Vision/Diagnostico", "string", 2, 3, 12),
    ),
    tab("Real x Modelo",
        *real_x_model("Shooter/Pivo", "Pivo", "Graus", 0, 0, ORANGE),
        *real_x_model("Shooter/Top", "Top", "Rpm", 4, 0, RED),
        *real_x_model("Shooter/Tower", "Tower", "Rpm", 8, 0, BLUE),
        text("Pivo: erro ao parar (graus)", "Shooter/Pivo/Comparacao/DiferencaAoPararGraus", "double", 0, 4, 2),
        text("Movimentos comparados", "Shooter/Pivo/Comparacao/MovimentosComparados", "int", 2, 4, 2),
    ),
    tab("Diagnostico",
        *motor_status("Shooter/Pivo", "Pivo", 0, 0),
        *motor_status("Shooter/Top", "Top", 0, 1),
        *motor_status("Shooter/Tower", "Tower", 0, 2),
        *motor_status("Centrifuga", "Centrifuga", 0, 3),
        alerts(8, 0, 6, 4),
    ),
)

ciclo = layout(
    tab("Ciclo da bola",
        text("Coletas (sensores)", "Fuel/Contagem/ColetasDetectadas", "int", 0, 0),
        text("Disparos (sensores)", "Fuel/Contagem/DisparosDetectados", "int", 2, 0),
        text("No deposito (estimado)", "Fuel/Contagem/NoDepositoEstimado", "int", 4, 0),
        boolean("Coleta agora", "Fuel/Contagem/ColetaAgora", 6, 0),
        boolean("Disparo agora", "Fuel/Contagem/DisparoAgora", 7, 0),
        text("Sim: coletadas", "Fuel/Simulacao/Coletadas", "int", 8, 0),
        text("Sim: lancadas", "Fuel/Simulacao/Lancadas", "int", 10, 0),
        text("Sim: acertos", "Fuel/Simulacao/Acertos", "int", 12, 0),
        graph("Rolete: corrente (A)", "Roller/CorrenteAmps", 0, 1, 5, 2, ORANGE, low=0, high=45),
        graph("Top (RPM)", "Shooter/Top/RpmEsquerdo", 5, 1, 5, 2, RED, low=0),
        graph("Intake (graus, 0 = aberto)", "Intake/AnguloSubidaGraus", 10, 1, 4, 2, GREEN, low=-5, high=100),
        boolean("Rolete puxando", "Roller/Puxando", 0, 3),
        boolean("Intake aberto", "Intake/Aberto", 1, 3),
        boolean("Centrifuga ligada", "Centrifuga/Ligada", 2, 3),
        text("Ultimo tiro (sim)", "Fuel/Simulacao/UltimoTiro/Resultado", "string", 3, 3, 3),
        text("Ultima anotacao (real)", "Fuel/Real/UltimaAnotacao", "string", 6, 3, 3),
        text("Real: acertos", "Fuel/Real/Acertos", "int", 9, 3),
        text("Real: erros", "Fuel/Real/Erros", "int", 11, 3),
    ),
    tab("Tiro e visao",
        text("Tiro numero", "Fuel/Tiro/Numero", "int", 0, 0),
        boolean("Planejado acerta", "Fuel/Tiro/Planejado/Acerta", 2, 0),
        boolean("Sensores: acerta", "Fuel/Tiro/Sensores/Acerta", 3, 0),
        text("Planejado: dist. do centro (m)", "Fuel/Tiro/Planejado/DistanciaDoCentroDoHubMetros", "double", 4, 0, 3),
        text("Sensores: dist. do centro (m)", "Fuel/Tiro/Sensores/DistanciaDoCentroDoHubMetros", "double", 7, 0, 3),
        text("Dif. velocidade (m/s)", "Fuel/Tiro/Diferenca/VelocidadeSaidaMps", "double", 10, 0, 2),
        text("Dif. direcao (graus)", "Fuel/Tiro/Diferenca/DirecaoGraus", "double", 12, 0, 2),
        boolean("Visao recente no tiro", "Fuel/Tiro/Visao/CorrecaoRecente", 0, 1, 2),
        boolean("Pose ja corrigida", "Fuel/Tiro/Visao/PoseCorrigidaDesdeReset", 2, 1, 2),
        text("Seg. desde a correcao", "Fuel/Tiro/Visao/SegundosDesdeCorrecao", "double", 4, 1),
        text("Camera", "Fuel/Tiro/Visao/UltimaCamera", "string", 6, 1),
        text("Erro da pose (m, sim)", "Fuel/Tiro/Visao/ErroDaPoseMetros", "double", 8, 1),
        text("Tiros com visao recente", "Fuel/Tiro/Visao/TirosComVisaoRecente", "int", 10, 1, 2),
        text("Tiros SEM visao recente", "Fuel/Tiro/Visao/TirosSemVisaoRecente", "int", 12, 1, 2),
        graph("Queda de RPM do top (deteccao do tiro)", "Fuel/Contagem/QuedaRpmTop", 0, 2, 7, 2, RED),
        graph("Idade da correcao da visao (s)", "Drive/Vision/CorrectionAgeSeconds", 7, 2, 7, 2, PURPLE, 20.0, 0, 3),
    ),
    tab("Real x Modelo",
        *real_x_model("Intake", "Intake", "Graus", 0, 0, GREEN),
        *real_x_model("Roller", "Rolete", "Rpm", 3, 0, ORANGE),
        *real_x_model("Centrifuga", "Centrifuga", "Rpm", 6, 0, PURPLE),
        *real_x_model("Shooter/Top", "Top", "Rpm", 9, 0, RED),
        graph("Rolete: corrente real - modelo (cada bola)", "Roller/Comparacao/DiferencaCorrenteAmps",
              0, 4, 6, 2, ORANGE),
    ),
    tab("Diagnostico",
        *motor_status("Intake", "Intake", 0, 0),
        *motor_status("Roller", "Rolete", 0, 1),
        *motor_status("Centrifuga", "Centrifuga", 0, 2),
        *motor_status("Shooter/Top", "Top", 0, 3),
        alerts(8, 0, 6, 4),
    ),
)

telescopio = layout(
    tab("Telescopio",
        text("Estado", "Telescopio/Estado", "string", 0, 0, 3),
        text("Comando", "Telescopio/Comando", "string", 3, 0, 3),
        boolean("Na meta", "Telescopio/NaMeta", 6, 0),
        boolean("Travado", "Telescopio/Protecao/Travado", 7, 0, good_when_true=False),
        boolean("Limite do topo", "Telescopio/Protecao/LimiteTopoAtivo", 8, 0, good_when_true=False),
        text("Modo", "Telescopio/Modo", "string", 9, 0, 3),
        graph("Posicao real (cm)", "Telescopio/Comparacao/RealCentimetros", 0, 1, 5, 2, ORANGE, low=0),
        graph("Corrente (A)", "Telescopio/Comparacao/RealCorrenteAmps", 5, 1, 5, 2, RED, low=0),
        text("Meta (m)", "Telescopio/MetaMetros", "double", 10, 1),
        text("Erro (m)", "Telescopio/ErroMetros", "double", 10, 2),
        text("Tensao pedida (V)", "Telescopio/TensaoSolicitadaVolts", "double", 12, 1),
    ),
    tab("Real x Modelo",
        *real_x_model("Telescopio", "Telescopio", "Centimetros", 0, 0, ORANGE),
        text("Erro ao parar (cm)", "Telescopio/Comparacao/DiferencaAoPararCentimetros", "double", 3, 0, 2),
        text("Movimentos comparados", "Telescopio/Comparacao/MovimentosComparados", "int", 3, 1, 2),
        graph("Corrente real - modelo (A)", "Telescopio/Comparacao/DiferencaCorrenteAmps", 5, 0, 5, 2, RED),
    ),
    tab("Diagnostico",
        *motor_status("Telescopio", "Telescopio", 0, 0),
        alerts(8, 0, 6, 4),
    ),
)

diagnostico = layout(
    tab("Pit - todos os motores",
        *motor_status("Shooter/Pivo", "Pivo", 0, 0),
        *motor_status("Shooter/Top", "Top", 0, 1),
        *motor_status("Shooter/Tower", "Tower", 0, 2),
        *motor_status("Centrifuga", "Centrifuga", 0, 3),
        *motor_status("Intake", "Intake", 0, 4),
        *motor_status("Roller", "Rolete", 0, 5),
        *motor_status("Telescopio", "Telescopio", 0, 6),
        alerts(7, 0, 7, 4),
        boolean("Visao corrigiu a pose", "Drive/Vision/CorrectedSinceReset", 7, 4, 2),
        boolean("Visao recente", "Drive/Vision/RecentCorrection", 9, 4, 2),
        boolean("Zero do pivo", "Shooter/Pivo/ZeroValido", 11, 4),
        boolean("Zero do intake", "Intake/ZeroValido", 12, 4),
    ),
)

LAYOUTS = {
    "FRC9460-Shooter": shooter,
    "FRC9460-CicloDaBola": ciclo,
    "Telescopio9460": telescopio,
    "FRC9460-Diagnostico": diagnostico,
}

# #endregion

def write_layouts():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    for name, data in LAYOUTS.items():
        path = OUTPUT_DIR / f"{name}.json"
        path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        widgets = sum(len(t["grid_layout"]["containers"]) for t in data["tabs"])
        print(f"{path.relative_to(ROOT)}: {len(data['tabs'])} abas, {widgets} widgets")


if __name__ == "__main__":
    write_layouts()
