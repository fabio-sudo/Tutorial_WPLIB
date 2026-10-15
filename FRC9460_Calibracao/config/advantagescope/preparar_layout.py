"""Gera um layout importável preservando as abas já salvas pelo usuário."""
from copy import deepcopy
from pathlib import Path
import json
import uuid

dest = Path(__file__).resolve().parent
saved = Path('C:/Users/Admin/AppData/Roaming/AdvantageScope/state-26_0_0.json')
original = json.loads(saved.read_text(encoding='utf-8-sig'))
original['version'] = '26.0.0'
backup = dest / 'Layout-anterior-2026-10-09.json'
if not backup.exists():
    backup.write_text(json.dumps(original, ensure_ascii=False, indent=2), encoding='utf-8')

layout = deepcopy(original)
hub = layout['hubs'][0]
state = hub['state']
rear, front, robot = '#3b875a', '#e48b32', '#2b66a2'

def camera(name, suffix):
    return '/RealOutputs/Vision/' + name + '/' + suffix

def source(key, color, log_type='Number'):
    return dict(type='stepped' if log_type == 'Number' else 'stripes',
                logKey=key, logType=log_type, visible=True,
                options=dict(color=color, **({'size': 'normal'} if log_type == 'Number' else {})))

def pair(suffix, log_type='Number'):
    return [source(camera(name, suffix), color, log_type)
            for name, color in [('Rear', rear), ('Front', front)]]

def tab(kind, title, controller, height=230, renderer=None):
    return dict(type=kind, title=title, controller=controller,
                controllerUUID=uuid.uuid4().hex, renderer=renderer, controlsHeight=height)

def graph(title, left, right=None, discrete=None):
    return tab(1, title, dict(leftSources=left, rightSources=right or [],
        discreteSources=discrete or [], leftLockedRange=None, rightLockedRange=None,
        leftUnitConversion=dict(autoTarget=None, preset=None),
        rightUnitConversion=dict(autoTarget=None, preset=None), leftFilter=0, rightFilter=0))

tabs = [
    graph('VISAO 1 - Deteccao', pair('MegaTag2/TagCount'),
          discrete=pair('Camera/Connected', 'Boolean') + pair('HasTarget', 'Boolean')),
    graph('VISAO 2 - Fusao', pair('MegaTag2/AcceptedCount'),
          discrete=pair('MegaTag2/Valid', 'Boolean')
          + pair('MegaTag2/NewMeasurement', 'Boolean')
          + pair('MegaTag2/MeasurementAccepted', 'Boolean')),
    tab(4, 'VISAO 3 - Resultado', [
        camera('Rear', 'MegaTag2/FusionResult'), camera('Front', 'MegaTag2/FusionResult'),
        camera('Rear', 'MegaTag2/RejectedCount'), camera('Front', 'MegaTag2/RejectedCount'),
        '/RealOutputs/Vision/Summary/ConfiguredCameras',
        '/RealOutputs/Vision/Summary/PendingFrames'], height=0),
    graph('VISAO 4 - Tempo', pair('Timing/CaptureAgeSeconds') + [
            source(camera('Rear', 'Timing/CaptureAgeLimitSeconds'), '#858584')],
          discrete=pair('MegaTag2/Reject/StaleMeasurement', 'Boolean')
          + pair('MegaTag2/Reject/FutureTimestamp', 'Boolean')
          + pair('GyroHealthy', 'Boolean')),
    graph('VISAO 5 - Confianca', pair('MegaTag2/XYStdDevMeters'),
          right=pair('MegaTag2/PoseErrorMeters'),
          discrete=pair('MegaTag2/Reject/CamerasDisagree', 'Boolean')
          + pair('MegaTag2/Reject/NoTags', 'Boolean')),
]

poses = [('/RealOutputs/Robot/Pose', robot),
         (camera('Rear', 'MegaTag2/Pose'), rear),
         (camera('Front', 'MegaTag2/Pose'), front)]
field = deepcopy(next(t for t in state['tabs']['tabs'] if t['type'] == 2)['controller'])
field['sources'] = [dict(type='ghost', logKey=k, logType='Pose2d', visible=True,
                         options=dict(color=c)) for k,c in poses]
tabs.append(tab(2, 'VISAO 6 - Campo 2D', field))
field3 = deepcopy(next(t for t in state['tabs']['tabs'] if t['type'] == 3)['controller'])
field3['sources'] = [dict(type='robot' if i == 0 else 'ghost', logKey=k,
                         logType='Pose2d', visible=True,
                         options=dict(model='2026 KitBot', **({} if i == 0 else {'color':c})))
                     for i,(k,c) in enumerate(poses)]
tabs.append(tab(3, 'VISAO 7 - Campo 3D', field3))

old_tabs = [t for t in state['tabs']['tabs'] if not t['title'].startswith('VISAO ')]
state['tabs']['tabs'] = old_tabs[:1] + tabs + old_tabs[1:]
state['tabs']['selected'] = 1
state['sidebar']['width'] = 300
for key in ['/RealOutputs/Vision', '/RealOutputs/Vision/Rear',
            '/RealOutputs/Vision/Rear/MegaTag2', '/RealOutputs/Vision/Front']:
    if key not in state['sidebar']['expanded']:
        state['sidebar']['expanded'].append(key)

output = dest / 'FRC9460-Visao.json'
output.write_text(json.dumps(layout, ensure_ascii=False, indent=2), encoding='utf-8')
assert len({t['controllerUUID'] for t in state['tabs']['tabs']}) == len(state['tabs']['tabs'])
print(f'Layout pronto: {len(tabs)} abas de visao, {len(old_tabs)} abas anteriores preservadas.')
print(output)
