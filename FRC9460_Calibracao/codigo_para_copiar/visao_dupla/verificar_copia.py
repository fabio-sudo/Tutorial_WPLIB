from pathlib import Path
import shutil

root = Path(__file__).resolve().parents[2]
proposal = Path(__file__).resolve().parent
stage = root / 'build' / 'verificacao_visao_dupla'
stage.mkdir(parents=True, exist_ok=True)
for directory in ('src', 'vendordeps', 'gradle', '.wpilib'):
    shutil.copytree(root / directory, stage / directory, dirs_exist_ok=True)
for name in ('build.gradle', 'settings.gradle', 'gradlew', 'gradlew.bat'):
    shutil.copy2(root / name, stage / name)
for filename, subsystem in (('Vision.java', 'vision'), ('Drive.java', 'drive')):
    shutil.copy2(proposal / filename,
                 stage / 'src/main/java/frc/robot/subsystems' / subsystem / filename)
testpath = stage / 'src/test/java/frc/robot/subsystems/vision'
testpath.mkdir(parents=True, exist_ok=True)
shutil.copy2(proposal / 'VisionMultiCameraTest.java', testpath / 'VisionMultiCameraTest.java')
print('Cópia isolada preparada. Fontes originais preservados.')
