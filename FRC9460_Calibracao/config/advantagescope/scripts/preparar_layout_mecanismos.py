"""Gera layouts/FRC9460-Mecanismos.json: um layout só para analisar os mecanismos.

Contém, para cada objeto de mecanismos/, a aba Mechanism (desenho 2D) e a aba
3D (GLBs), e gráficos dos movimentos. Importar este layout substitui as abas
abertas; para voltar às demais, importe layouts/FRC9460-Simulacao3D.json.
"""

from copy import deepcopy
import json
import uuid

from preparar_simulacao_3d import DEST, ROOT, SAVED, graph, mechanism_tabs

OUTPUT = DEST / "layouts" / "FRC9460-Mecanismos.json"


def curve(key, color):
    return dict(type="smooth", logKey=ROOT + key, logType="Number", visible=True,
                options=dict(color=color, size="normal"))


def analysis_graphs():
    return [
        graph("Robo 9460 - Angulos", [curve("Mecanismos/IntakeGraus", "#ff8c00"),
                                      curve("Mecanismos/TorreGraus", "#00c0ff")]),
        graph("Telescopio 9460 - Curso", [curve("Telescopio/ExtensaoN2Metros", "#ff8c00"),
                                          curve("Telescopio/ExtensaoRelativaN3Metros", "#00cc66"),
                                          curve("Telescopio/ExtensaoVisualMetros", "#33bbff")]),
        graph("Telescopio 9458 - Curso", [curve("Telescopio9458/ExtensaoTubo3Metros", "#ff9900"),
                                          curve("Telescopio9458/ExtensaoTubo2Metros", "#44dd88"),
                                          curve("Telescopio9458/ExtensaoMetros", "#33bbff"),
                                          curve("Telescopio9458/AlturaTotalMetros", "#ffffff")],
              right=[curve("Telescopio9458/MotorRotacoes", "#ff4466")]),
    ]


def main():
    base = json.loads(SAVED.read_text(encoding="utf-8-sig"))
    hub = deepcopy(base["hubs"][0])  # mantém tamanho e posição da janela
    documentation = dict(type=0, title="", controller=None, controllerUUID=uuid.uuid4().hex,
                         renderer="", controlsHeight=0)
    tabs = [documentation] + mechanism_tabs() + analysis_graphs()
    selected = next(i for i, t in enumerate(tabs) if t["title"] == "Telescopio 9458 3D")
    hub["state"] = dict(
        sidebar=dict(width=300, expanded=[ROOT + "Mecanismos", ROOT + "Telescopio", ROOT + "Telescopio9458"]),
        tabs=dict(selected=selected, tabs=tabs))
    layout = dict(hubs=[hub], satellites=[], version="26.0.0")
    OUTPUT.write_text(json.dumps(layout, ensure_ascii=False, indent=2), encoding="utf-8")
    assert len({t["controllerUUID"] for t in tabs}) == len(tabs)
    print(f"Layout: {OUTPUT}")
    print("Abas:", ", ".join(t["title"] or "(documentação)" for t in tabs))


if __name__ == "__main__":
    main()
