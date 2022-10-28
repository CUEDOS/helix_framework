import csv
import atexit
from datetime import datetime
from tkinter import E


class CSVLogger:
    def __init__(self):
        self.running = True
        atexit.register(self.stop)

    def write_log(self, queue, drone_id, experiment_path):
        self.running = True
        experiment_name = experiment_path.split("/")[-1]
        print(experiment_name)
        file_name = datetime.now().strftime(
            "logs/" + experiment_name + "_" + drone_id + "_log-%Y-%m-%d-%H-%M.csv"
        )
        with open(file_name, "a+") as self.f:
            writer = csv.writer(self.f, delimiter=",", quoting=csv.QUOTE_NONE)
            writer.writerow(
                (
                    "time",
                    "GPS_time",
                    "loopback_time",
                    "next_point",
                    "position_N",
                    "position_E",
                    "position_D",
                    "velocity_N",
                    "velocity_E",
                    "velocity_D",
                    "N_v_migration",
                    "E_v_migration",
                    "D_v_migration",
                    "N_v_lane_cohesion",
                    "E_v_lane_cohesion",
                    "D_v_lane_cohesion",
                    "N_v_separation",
                    "E_v_separation",
                    "D_v_separation",
                    "N_v_rotation",
                    "E_v_rotation",
                    "D_v_rotation",
                    "N_v_force_field",
                    "E_v_force_field",
                    "D_v_force_field",
                )
            )
            while self.running:
                if queue.empty():
                    continue
                csv_line = queue.get()
                writer.writerow(csv_line)
            self.f.close()

    def stop(self):
        self.running = False
        # if hasattr(self, "f"):
        #     self.f.close()
