import csv
import atexit


class CSVLogger:
    def __init__(self):
        self.running = True
        atexit.register(self.stop)

    def write_log(self, queue):
        with open("log.csv", "a") as self.f:
            # dict_writer = csv.DictWriter(self.f, fieldnames=self.field_names)
            writer = csv.writer(self.f, delimiter=",", quoting=csv.QUOTE_NONE)
            writer.writerow(
                (
                    "time",
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
                # with open("log.csv", "a") as f:
                # dict_writer = csv.DictWriter(f, fieldnames=self.field_names)
                csv_line = queue.get()
                writer.writerow(csv_line)
                # f.close

    def stop(self):
        self.running = False
        if hasattr(self, "f"):
            self.f.close()
