#!/usr/bin/env python

import matplotlib.pyplot as plt
import csv

times, measures, estimates = [], [], []
with open('log.csv', 'r') as csvfile:
    log = csv.reader(csvfile, delimiter=',')
    next(log)  # skip header row

    for row in log:
        times.append(float(row[0]))
        measures.append((float(row[1])))
        estimates.append(float(row[2]))

plt.plot(times, measures, label="Measures [m]")
plt.plot(times, estimates, label="Estimates [m]")
plt.xlabel('Time [s]')
plt.grid()
plt.legend()
plt.show()
