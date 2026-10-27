"""
Converte o CSV exportado pelo AdvantageScope (robô real) num .wpilog para o SysId
e calcula os ganhos com o mesmo modelo do SysId (Elevator).

Os trechos de teste vêm de Telescopio9458/Estado == "SYSID" dentro de cada comando:
no comando QUASISTATICO/DINAMICO, o 1º trecho é a subida (forward) e o 2º a descida (reverse).
Trechos com menos de 1 s (botão solto logo no começo) são ignorados.

Uso: python -I sysid_9458_csv_para_wpilog.py entrada.csv saida.wpilog
(CSV: AdvantageScope > File > Export Data, com a gravação ao vivo do robô)
"""
import csv
import struct
import sys

csv.field_size_limit(10 ** 9)
src, dst = sys.argv[1], sys.argv[2]
reader = csv.reader(open(src, newline="", encoding="utf-8", errors="replace"))
head = next(reader)
col = {name: head.index(name) for name in (
    "/RealOutputs/Telescopio9458/Estado", "/RealOutputs/Telescopio9458/Comando",
    "/Telescopio9458/Inputs/ExtensionMeters", "/Telescopio9458/Inputs/VelocityMetersPerSec",
    "/Telescopio9458/Inputs/AppliedVolts")}


def num(row, name):
    try:
        return float(row[col[name]])
    except (ValueError, IndexError):
        return None


samples = []  # (t, estado, comando, pos, vel, volts)
for row in reader:
    t = float(row[0])
    samples.append((t, row[col["/RealOutputs/Telescopio9458/Estado"]], row[col["/RealOutputs/Telescopio9458/Comando"]],
                    num(row, "/Telescopio9458/Inputs/ExtensionMeters"),
                    num(row, "/Telescopio9458/Inputs/VelocityMetersPerSec"),
                    num(row, "/Telescopio9458/Inputs/AppliedVolts")))

# Trechos SYSID consecutivos
segments, cur = [], None
for k, s in enumerate(samples):
    if s[1] == "SYSID":
        if cur is None:
            cur = [k, k, s[2]]
        cur[1] = k
    elif cur is not None:
        segments.append(cur)
        cur = None

# Nome do teste: tipo pelo comando, sentido pela ordem dentro do mesmo aperto
tests, previous_cmd_start = [], None
for a, b, cmd in segments:
    kind = "quasistatic" if "QUASISTATICO" in cmd else "dynamic" if "DINAMICO" in cmd else None
    duration = samples[b][0] - samples[a][0]
    # Mesmo aperto: o comando não voltou a NENHUM entre os dois trechos
    same_press = previous_cmd_start is not None and all(
        samples[k][2] == cmd for k in range(previous_cmd_start, a))
    direction = "reverse" if same_press else "forward"
    previous_cmd_start = a
    if kind is None or duration < 1.0:
        print("ignorado: %.2f-%.2f s (%.2f s) %s" % (samples[a][0], samples[b][0], duration, cmd))
        continue
    # Confere o sentido pela tensão
    volts = [samples[k][5] for k in range(a, b + 1) if samples[k][5] is not None]
    assert (sum(volts) > 0) == (direction == "forward"), "sentido não confere com a tensão"
    tests.append((a, b, "%s-%s" % (kind, direction)))
    print("teste: %-20s %.2f-%.2f s (%.2f s), %d amostras" % (
        "%s-%s" % (kind, direction), samples[a][0], samples[b][0], duration, b - a + 1))

# ------------------------------------------------------------------ escreve o .wpilog
out = bytearray(b"WPILOG" + struct.pack("<H", 0x0100) + struct.pack("<I", 0))


def record(entry, t, payload):
    out.append(0x7F)  # id 4 bytes, tamanho 4 bytes, timestamp 8 bytes
    out.extend(struct.pack("<IIQ", entry, len(payload), int(round(t * 1e6))))
    out.extend(payload)


def start(entry, name, typ):
    n, ty = name.encode(), typ.encode()
    record(0, 0, b"\x00" + struct.pack("<I", entry) + struct.pack("<I", len(n)) + n
           + struct.pack("<I", len(ty)) + ty + struct.pack("<I", 0))


STATE, POS, VEL, VOLT = 1, 2, 3, 4
start(STATE, "/Telescopio9458/SysIdTestState", "string")
start(POS, "/Telescopio9458/ExtensionMeters", "double")
start(VEL, "/Telescopio9458/VelocityMetersPerSec", "double")
start(VOLT, "/Telescopio9458/AppliedVolts", "double")

state_at = {}
for a, b, name in tests:
    state_at[a] = name
    state_at[b + 1] = "none"
record(STATE, samples[0][0], b"none")
for k, s in enumerate(samples):
    t = s[0]
    if k in state_at and k > 0:
        record(STATE, t, state_at[k].encode())
    for entry, value in ((POS, s[3]), (VEL, s[4]), (VOLT, s[5])):
        if value is not None:
            record(entry, t, struct.pack("<d", value))
open(dst, "wb").write(out)
print("gravado:", dst, len(out), "bytes")

# ------------------------------------------------------------------ ajuste (modelo Elevator do SysId)
# V = kS·sinal(v) + kG + kV·v + kA·a, só com amostras em movimento dentro dos testes.


def fit(selected):
    normal = [[0.0] * 5 for _ in range(4)]
    count = 0
    for a, b, _ in selected:
        for k in range(a + 1, b + 1):
            v, v0, volts = samples[k][4], samples[k - 1][4], samples[k][5]
            dt = samples[k][0] - samples[k - 1][0]
            if None in (v, v0, volts) or dt <= 0 or abs(v) < 0.02:
                continue
            x = [1.0 if v > 0 else -1.0, 1.0, v, (v - v0) / dt]
            for r in range(4):
                for c in range(4):
                    normal[r][c] += x[r] * x[c]
                normal[r][4] += x[r] * volts
            count += 1
    for p in range(4):
        for r in range(p + 1, 4):
            f = normal[r][p] / normal[p][p]
            for c in range(p, 5):
                normal[r][c] -= f * normal[p][c]
    k = [0.0] * 4
    for r in range(3, -1, -1):
        s = normal[r][4] - sum(normal[r][c] * k[c] for c in range(r + 1, 4))
        k[r] = s / normal[r][r]
    # R² e erro médio
    ss_res = ss_tot = total = 0.0
    ys = []
    for a, b, _ in selected:
        for kk in range(a + 1, b + 1):
            v, v0, volts = samples[kk][4], samples[kk - 1][4], samples[kk][5]
            dt = samples[kk][0] - samples[kk - 1][0]
            if None in (v, v0, volts) or dt <= 0 or abs(v) < 0.02:
                continue
            pred = k[0] * (1 if v > 0 else -1) + k[1] + k[2] * v + k[3] * (v - v0) / dt
            ss_res += (volts - pred) ** 2
            ys.append(volts)
    mean = sum(ys) / len(ys)
    ss_tot = sum((y - mean) ** 2 for y in ys)
    return k, count, 1 - ss_res / ss_tot, (ss_res / len(ys)) ** 0.5


for label, chosen in (("todos os testes", tests),
                      ("so quasistatico", [t for t in tests if t[2].startswith("quasistatic")]),
                      ("so dinamico", [t for t in tests if t[2].startswith("dynamic")])):
    k, n, r2, rmse = fit(chosen)
    print("%-16s kS=%.4f V  kG=%.4f V  kV=%.3f V/(m/s)  kA=%.4f V/(m/s^2)  amostras=%d  R2=%.4f  erro=%.3f V" % (
        label, k[0], k[1], k[2], k[3], n, r2, rmse))
