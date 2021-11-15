import pandas
import numpy as np
import matplotlib.pyplot as plt

colnames = ["time_of_flight", "north", "east", "down"]
data = pandas.read_csv("range_test_data.csv", names=colnames)

time_of_flight = np.array(data.time_of_flight.tolist())
north = np.array(data.north.tolist())
east = np.array(data.east.tolist())
down = np.array(data.down.tolist())

distance = np.sqrt(north ** 2 + east ** 2 + down ** 2)
plt.scatter(distance, time_of_flight, 0.5)
plt.xlabel("Distance from Router [m]")
plt.ylabel("Time from send to receive message [s]")
plt.show()
