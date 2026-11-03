"""Acrescenta as abas REAL x MODELO e DIAGNÓSTICO aos layouts do telescópio 9460.

Os dois layouts (Telescopio9460-Simulacao e Telescopio9460-TesteControle) foram
montados à mão. Este script só troca as abas que ele mesmo cria (pelo título),
então pode rodar quantas vezes quiser sem duplicar nada.

Uso, na raiz do projeto: python config/advantagescope/scripts/adicionar_real_x_modelo_telescopio_9460.py
"""

import json
import uuid
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent  # config/advantagescope
LAYOUTS = [BASE / "layouts" / "Telescopio9460-Simulacao.json",
           BASE / "layouts" / "Telescopio9460-TesteControle.json"]

REAL = "#e48b32"     # laranja = telescópio (medido)
MODEL = "#00bcd4"    # ciano = modelo em paralelo (física com a tensão medida)
LIMIT = "#e53935"
NEUTRAL = "#c0c0c0"
PREFIX = "Telescopio"
TITLES = ("Real x modelo - posicao", "Real x modelo - corrente", "Diagnostico do motor")


def out(key):
    return "/RealOutputs/" + key


def line(key, color, kind="smooth"):
    return dict(type=kind, logKey=key, logType="Number", visible=True,
                options=dict(color=color, size="normal"))


def stripes(key, color, log_type="Boolean"):
    return dict(type="stripes", logKey=key, logType=log_type, visible=True, options=dict(color=color))


def tab(kind, title, controller, height=230):
    return dict(type=kind, title=title, controller=controller, controllerUUID=uuid.uuid4().hex,
                renderer=None, controlsHeight=height)


def graph(title, left, right=(), discrete=()):
    return tab(1, title, dict(
        leftSources=list(left), rightSources=list(right), discreteSources=list(discrete),
        leftLockedRange=None, rightLockedRange=None,
        leftUnitConversion=dict(autoTarget=None, preset=None),
        rightUnitConversion=dict(autoTarget=None, preset=None), leftFilter=0, rightFilter=0))


c = out(PREFIX + "/Comparacao/")
new_tabs = [
    graph(TITLES[0],
          [line(c + "RealCentimetros", REAL), line(c + "SimuladoCentimetros", MODEL)],
          [line(c + "DiferencaCentimetros", NEUTRAL), line(c + "ToleranciaCentimetros", LIMIT, "stepped"),
           line(c + "DiferencaAoPararCentimetros", "#ba68c8", "stepped")],
          [stripes(out(PREFIX + "/Modelo/ComparacaoAtiva"), MODEL),
           stripes(c + "Divergindo", LIMIT),
           stripes(out(PREFIX + "/Estado"), NEUTRAL, "String"),
           stripes(out(PREFIX + "/Diagnostico/Resultado"), NEUTRAL, "String")]),
    graph(TITLES[1],
          [line(c + "RealCorrenteAmps", REAL), line(c + "SimuladoCorrenteAmps", MODEL)],
          [line(c + "RealCentimetrosPorSeg", "#ffcc80"), line(c + "SimuladoCentimetrosPorSeg", "#b2ebf2")],
          [stripes(out(PREFIX + "/Protecao/Travado"), LIMIT),
           stripes(c + "Divergindo", LIMIT)]),
    tab(4, TITLES[2], [out(PREFIX + "/Diagnostico/" + k) for k in ("Execucao", "MotorRespondendo", "Resultado")]
        + [out(PREFIX + "/Modo"), out(PREFIX + "/Comando"),
           c + "DiferencaMaximaCentimetros", c + "DiferencaAoPararCentimetros",
           c + "MovimentosComparados", c + "Divergindo"], height=0),
]

for path in LAYOUTS:
    layout = json.loads(path.read_text(encoding="utf-8"))
    tabs = layout["hubs"][0]["state"]["tabs"]["tabs"]
    tabs[:] = [t for t in tabs if t["title"] not in TITLES] + new_tabs
    path.write_text(json.dumps(layout, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(path.name, [t["title"] for t in tabs])
