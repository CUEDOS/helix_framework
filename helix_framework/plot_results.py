import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
import numpy as np

drone_log = pd.read_csv("logs/S002_log_-2022-10-21-13-13.csv")

v_separation = np.linalg.norm(
    drone_log[["N_v_separation", "E_v_separation", "D_v_separation"]].values, axis=1
)

drone_log.loc[:, "N_v_separation"]

plot_data = pd.DataFrame(
    data={"time": drone_log.loc[:, "time"], "v_seperation": v_separation}
)

sns.relplot(data=plot_data, x="time", y="v_seperation")
plt.show()
