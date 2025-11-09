# This file isn’t part of the solution. it’s only used for PID controller tuning and overall testing.
import matplotlib.pyplot as plt

log_path = "/home/hubert/worskspace/pwr/scorpio_zadanie_rekrutacyjne_software/src/solution/motor_log.txt"

set1 = []
set2 = []
segments = []  

pair_index = 0
current_consts = None
current_start = 1

with open(log_path, "r") as f:
    for line in f:
        parts = line.strip().split()
        if not parts:
            continue
        if len(parts) == 2:
            c1 = float(parts[0])
            c2 = float(parts[1])
            if current_consts is not None:
                segments.append((current_start, pair_index, current_consts[0], current_consts[1]))
            current_consts = (c1, c2)
            current_start = pair_index + 1 
        else:
            val = float(parts[0])
            if len(set1) == len(set2):
                set1.append(val)
            else:
                set2.append(val)
                pair_index += 1  

if current_consts is not None:
    segments.append((current_start, pair_index, current_consts[0], current_consts[1]))


x_pairs = list(range(1, pair_index + 1))

plt.figure(figsize=(12, 6))

if set1:
    x1 = list(range(1, len(set1) + 1))
    line1, = plt.plot(x1, set1, label='Set 1', linewidth=1.2)
    c1_color = line1.get_color()
else:
    c1_color = 'r'

if set2:
    x2 = list(range(1, len(set2) + 1))
    line2, = plt.plot(x2, set2, label='Set 2', linewidth=1.2)
    c2_color = line2.get_color()
else:
    c2_color = 'b'

for start, end, const1, const2 in segments:
    if end >= start and end > 0:
        plt.hlines(const1, xmin=start, xmax=end, colors=c1_color, linestyles='--')
        plt.hlines(const2, xmin=start, xmax=end, colors=c2_color, linestyles='--')

plt.grid(True, linestyle='-', alpha=0.5)
plt.legend()
plt.show()