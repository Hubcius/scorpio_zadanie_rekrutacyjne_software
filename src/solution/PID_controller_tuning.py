# This file isn’t part of the solution. it’s only used for PID controller tuning and overall testing. The same applies to motor_log.txt.
import matplotlib.pyplot as plt
import numpy as np

data = np.loadtxt("/home/hubert/worskspace/pwr/scorpio_zadanie_rekrutacyjne_software/src/solution/motor_log.txt")

const1 = data[0]
const2 = data[1]

rest_data = data[2:]
set1 = rest_data[::2]
set2 = rest_data[1::2]
x_rest = list(range(1, 1 + len(set1))) 

plt.figure(figsize=(12,6))
plt.hlines(const1, xmin=x_rest[0], xmax=x_rest[-1], colors='r', linestyles='--', label='Const 1')
plt.hlines(const2, xmin=x_rest[0], xmax=x_rest[-1], colors='b', linestyles='--', label='Const 2')
plt.plot(x_rest, set1, label='Set 1', linewidth=1.2)
plt.plot(x_rest, set2, label='Set 2', linewidth=1.2)
plt.grid(True, linestyle='-', alpha=0.5)
plt.show()
