# Validação do replay da visão — 15/10/2026

## Coleta analisada

- Original: `logs/akit_26-10-15_12-29-31.wpilog`.
- Replay: `logs/akit_26-10-15_12-29-31_replay.wpilog`.
- Origem: simulação, com observações PhotonLib das duas câmeras.
- Duração do original: 191,515 segundos.
- Vídeo conferido: `Gravando 2026-10-15 124540.mp4`.

O original é um WPILOG válido do AdvantageKit e contém inputs dos quatro
módulos, do gyro, da inicialização do Drive e das duas câmeras. O replay foi
executado com sucesso, em modo REPLAY e sem extensões HAL. O hash do arquivo
original permaneceu igual após a execução.

## Comparação das poses

Foram comparadas as poses `/RealOutputs/Robot/Pose` e
`/ReplayOutputs/Robot/Pose` nos mesmos ciclos do arquivo resultante.
Os valores registrados são mantidos entre atualizações, conforme a gravação
incremental do WPILOG. Ciclos sem ambas as poses disponíveis não são comparados.

| Verificação | Resultado |
| --- | --- |
| Ciclos com ambas as poses | 8.822 |
| Diferença máxima em X/Y | 0 m |
| Diferença RMS em X/Y | 0 m |
| Diferença angular máxima, normalizada | 0° |
| Ciclos com diferença acima de 1e-9 m/rad | 0 |

O vídeo confirma a configuração da visualização: o modelo usa a pose recalculada
como Robot e a pose original como Ghost verde, ambos com o modelo FRC9460.
Eles permanecem sobrepostos nos trechos observados de movimento e rotação.
A comparação numérica abrange toda a janela com ambas as poses disponíveis.

## Contadores finais da fusão

| Câmera | Aceitas: original / replay | Rejeitadas: original / replay |
| --- | --- | --- |
| Rear | 5.403 / 5.403 | 314 / 314 |
| Front | 5.496 / 5.496 | 125 / 125 |

O arquivo de replay termina aproximadamente um ciclo antes do original devido
ao encerramento da leitura no fim do arquivo. A comparação considera os ciclos
com dados disponíveis; os contadores finais acima coincidiram.

## Conclusão e alcance

A gravação dos inputs e a reprodução da fusão estão validadas para esta coleta,
com o código usado na execução do replay. A diferença zero era o comportamento
esperado ao repetir os mesmos inputs com a mesma lógica.

Essa diferença mede concordância entre a execução original e o replay. Não mede
erro físico, precisão da Limelight, fidelidade óptica ou derrapagem do robô.
Essas verificações continuam dependendo de referências independentes e dos
testes físicos previstos.

Esta sessão pode servir de referência para comparar futuras alterações dos
filtros e dos pesos da visão. Mudanças na óptica ou na montagem exigem uma nova
simulação/coleta, pois o replay reutiliza as poses de câmera já calculadas.
