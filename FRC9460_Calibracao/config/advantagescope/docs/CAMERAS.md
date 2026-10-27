# Teste das câmeras no AdvantageScope

Layout: `layouts/FRC9460-Cameras.json` (**File > Import Layout...**).
Gerador: `scripts/preparar_layout_cameras.py`.

- As abas **CAM** usam as chaves do `Vision`, iguais no robô real (Limelight) e na
  simulação (PhotonVision).
- As abas **SIM CAM** só têm dados na simulação.
- Cores: **verde = Rear** (`limelight-rear`), **laranja = Front** (`limelight-front`),
  **ciano = referência da simulação**, **vermelho = rejeitado ou limite**.

## Câmeras virtuais (simulação)

1. No VS Code, **WPILib: Simulate Robot Code** com a **Simulation GUI**.
2. No AdvantageScope, **File > Connect to Simulator > NetworkTables 4 (AdvantageKit)**.
3. Importe o layout. Na Simulation GUI, coloque em **Teleoperated** e **Enabled**
   e dirija o robô (o controle vai em **Joysticks [0]**) apontando as câmeras para as tags.
4. Depois, o mesmo teste pode ser revisto em `logs/simulacao/Simulacao_*.wpilog`
   com **File > Open Log(s)...**.

Na simulação as duas câmeras existem, com o perfil nominal da Limelight 4.

## Câmeras reais (Limelight)

1. Computador na rede do robô; **File > Connect to Robot > NetworkTables 4 (AdvantageKit)**
   (endereço `10.94.60.2`).
2. Importe o layout e aponte a câmera para uma AprilTag do campo.
3. Hoje só a **Rear** é criada no robô (`VisionConstants.FRONT_CAMERA_ENABLED = false`):
   as séries Front ficam vazias até essa constante ser ligada.
4. Para conferir a câmera sem o código do robô, use a página da Limelight
   (`http://limelight-rear.local:5801`). Os valores brutos da Limelight não aparecem
   na árvore do AdvantageKit; o layout mostra o que o código leu (`/Vision/Rear/Inputs`).

## O que olhar em cada aba

| Aba | Real e simulação | Esperado |
| --- | --- | --- |
| CAM 1 - Campo 3D | Robô, poses MegaTag2 de cada câmera e poses rejeitadas (vermelho). Na simulação: referência (ciano), tags vistas e eixos das câmeras. | A pose da câmera fica sobre o robô quando há tags. |
| CAM 2 - Campo 2D | O mesmo, visto de cima. | — |
| CAM 3 - Conexao e tags | Tags por câmera; `Heartbeat` (eixo direito); faixas `Connected` e `HasTarget`. | `Heartbeat` sempre subindo; `Connected` contínuo. |
| CAM 4 - Tempo | Latência informada e idade da captura, com o limite em vermelho; faixas de rejeição por medição antiga ou futura. | Idade abaixo do limite. |
| CAM 5 - Fusao | Medições aceitas e rejeitadas; faixas `MeasurementAccepted` e `FusionResult`. | Aceitas crescendo com tags à vista. |
| CAM 6 - Qualidade | Distância média das tags, erro de pose, desvio X/Y e diferença entre as câmeras (com limite). | Erro pequeno; diferença abaixo do limite. |
| CAM - Rejeicoes Rear/Front | Uma faixa por motivo de rejeição (16), com os contadores. | Mostra por que uma medição foi descartada. |
| CAM - Entradas Rear/Front | Tabela do que o IO leu: TX, TY, TA, ID, tags, latência, pose. | Na Limelight real, TX/TY mudam ao mover a tag; na simulação eles não são preenchidos. |

| Aba | Só simulação |
| --- | --- |
| SIM CAM - Visao da camera Rear/Front | Vista em primeira pessoa da câmera virtual (`cameraOverride`), com as tags que ela detecta. Para voltar à órbita, oculte a fonte `CameraPose`. |
| SIM CAM - Erro x referencia | Erro de posição e de rotação da estimativa de cada câmera contra a pose de referência; faixa de multi-tag. |
| SIM CAM - Tags vistas | Tags detectadas e tags usadas na estimativa; faixa `HasTargets`. |

## Checagem rápida da câmera real

1. **Conexão:** `Connected` verde contínuo e `Heartbeat` subindo (CAM 3).
2. **Detecção:** com uma tag à vista, `HasTarget` liga, `TagId` mostra o número
   e TX/TY mudam ao girar o robô (CAM - Entradas Rear).
3. **Pose:** a pose MegaTag2 aparece perto do robô no campo (CAM 1); se não aparecer,
   o motivo está em **CAM - Rejeicoes Rear**.
4. **Tempo:** latência estável e idade da captura abaixo do limite (CAM 4).

Para regenerar o layout, na raiz do projeto:

```powershell
python config/advantagescope/scripts/preparar_layout_cameras.py
```
