#!/usr/bin/env python3

# /home/r32401vc/anaconda3/bin/python3


import sys
import subprocess
import threading
from tkinter.constants import N
import mavsdk
import os
import signal
import asyncio
import gtools
import time
import tkinter as tk
from tkinter import filedialog
from tkinter import messagebox
import paho.mqtt.client as mqtt
from communication import GroundCommunication


class App:
    def __init__(self, master):

        self.master = master
        self.experiment_running = False
        self.normal_button_colour = "grey"
        self.activated_button_colour = "#4E73ED"
        self.check_var = tk.IntVar(value=1)
        self.flocking_type = tk.StringVar(value="Select a flocking type")
        self.flocking_options = [
            "Simple Flocking",
            "Migration Test",
            "Advanced Flocking",
            "Another type of Flocking",
        ]
        self.gazebo_process = None
        self.mavsdk_process = [None]
        self.script_process = [None]

        # This is the section of code which creates the main window
        master.rowconfigure(tuple(range(2)), minsize=100)
        master.grid_columnconfigure(tuple(range(2, 5)), uniform="button", minsize=146)

        # Create the frame with warnings
        self.warning_frame = tk.Frame(master)
        self.warning_frame.grid(row=5, column=2, columnspan=3, sticky="ew")

        # Create the frame with the status of the agents
        self.status_frame = tk.Frame(master)
        self.status_frame.grid(row=0, column=1, rowspan=5, sticky="nesw")

        # Create the frame containing sitl controls
        self.sitl_frame = tk.Frame(master)
        self.sitl_frame.columnconfigure(tuple(range(3)), weight=1)
        self.sitl_frame.grid(row=4, column=2, columnspan=3, sticky="ew")

        # Create the frame containing relaunch controls
        self.relaunch_frame = tk.Frame(self.sitl_frame)
        self.relaunch_frame.columnconfigure(tuple(range(2)), weight=1)
        self.relaunch_frame.grid(row=3, column=0, columnspan=3, sticky="ew")

        self.launch_btn = tk.Button(
            self.sitl_frame,
            text="Launch SITL Simulation",
            bg="#008B45",
            font=("arial", 12, "normal"),
            command=self.on_click_launch,
        )
        self.launch_btn.grid(row=2, column=0, columnspan=3, sticky="e")

        self.select_firmware_btn = tk.Button(
            self.sitl_frame,
            text="Select",
            # bg="#008B45",
            font=("arial", 12, "normal"),
            command=self.on_click_select,
        )
        self.select_firmware_btn.grid(row=1, column=2, sticky="e")

        self.start_button = tk.Button(
            master,
            text="Start",
            bg=self.normal_button_colour,
            font=("arial", 12, "normal"),
            command=self.on_click_start,
        )
        self.start_button.grid(
            row=0,
            column=4,
            sticky="nesw",
        )

        self.hold_button = tk.Button(
            master,
            text="Hold",
            bg=self.normal_button_colour,
            font=("arial", 12, "normal"),
            command=self.on_click_hold,
        )
        self.hold_button.grid(row=1, column=2, sticky="nesw")

        self.takeoff_button = tk.Button(
            master,
            text="Take Off",
            bg=self.normal_button_colour,
            font=("arial", 12, "normal"),
            command=self.on_click_takeoff,
        )
        self.takeoff_button.grid(row=0, column=3, sticky="nesw")

        self.land_button = tk.Button(
            master,
            text="Land",
            bg=self.normal_button_colour,
            font=("arial", 12, "normal"),
            command=self.on_click_land,
        )
        self.land_button.grid(row=1, column=3, sticky="nesw")

        tk.Button(
            master,
            text="Arm",
            bg="#CD4F39",
            font=("arial", 12, "normal"),
            command=self.on_click_arm,
        ).grid(row=0, column=2, sticky="nesw")

        self.return_button = tk.Button(
            master,
            text="Return",
            bg=self.normal_button_colour,
            font=("arial", 12, "normal"),
            command=self.on_click_return,
        )
        self.return_button.grid(row=1, column=4, sticky="nesw")

        self.flocking_menu = tk.OptionMenu(
            master, self.flocking_type, *self.flocking_options
        )
        self.flocking_menu.grid(row=3, column=2, columnspan=2, sticky="w")

        self.sitl_check = tk.Checkbutton(
            master,
            text="SITL",
            variable=self.check_var,
            onvalue=1,
            offvalue=0,
            command=self.sitl_check,
        )
        self.sitl_check.grid(row=3, column=4, sticky="e")

        tk.Button(
            master,
            text="Confirm",
            command=self.on_click_confirm,
        ).grid(row=2, column=4, sticky="nesw")

        self.real_drones_label = tk.Label(master, text="Real Drones:")
        self.real_drones_label.grid(row=2, column=2, sticky="w")

        self.real_drones_entry = tk.Entry(master)
        self.real_drones_entry.insert(tk.END, "0")
        self.real_drones_entry.grid(row=2, column=3, sticky="w")

        self.sitl_drones_label = tk.Label(self.sitl_frame, text="SITL Drones:")
        self.sitl_drones_label.grid(row=0, column=0, sticky="w")

        self.sitl_drones_entry = tk.Entry(self.sitl_frame)
        self.sitl_drones_entry.insert(tk.END, "3")
        self.sitl_drones_entry.grid(row=0, column=1, sticky="w")

        self.firmware_label = tk.Label(self.sitl_frame, text="PX4 Firmware Path:")
        self.firmware_label.grid(row=1, column=0, sticky="w")

        self.path_label = tk.Label(self.sitl_frame)
        # change back file path after debugging
        with open("config.txt") as f:
            self.firmware_path = f.read()
        self.path_label.config(text=self.firmware_path)
        self.path_label.grid(row=1, column=1, sticky="w")

        tk.Button(
            self.relaunch_frame,
            text="relaunch Gazebo",
            command=self.on_click_relaunch_gazebo,
        ).grid(row=0, column=0, sticky="nesw")

        tk.Button(
            self.relaunch_frame,
            text="relaunch scripts",
            command=self.on_click_relaunch_scripts,
        ).grid(row=0, column=1, sticky="nesw")

        self.warning_title = tk.Label(
            self.warning_frame, text="WARNING", bg="red", font=("arial", 20, "normal")
        )
        self.warning_title.grid(row=0, column=0, sticky="nsw")

        self.warning_label = tk.Label(self.warning_frame, text="")
        self.warning_label.grid(row=0, column=1, sticky="w")
        self.warning_frame.grid_remove()

        # begin communications with default swarm size
        # self.real_swarm_size = 0
        # self.sitl_swarm_size = 3
        # self.swarm_size = self.real_swarm_size + self.sitl_swarm_size
        # self.empty_swarm_dict = gtools.create_swarm_dict(
        #     self.real_swarm_size, self.sitl_swarm_size
        # )
        self.initialise_swarm()
        self.comms_thread = None
        self.start_comms_thread()

        # self.sitl_drone_ids = range(101, 101 + self.swarm_size)

        # create default status elements
        self.status_element_frame = {}
        self.generate_status_elements()

        # Set up window title and icon
        master.title("Helix.io")
        master.iconphoto(True, tk.PhotoImage(file="../img/HelixioLogoFinalSmall.png"))
        master.protocol("WM_DELETE_WINDOW", self.on_closing)
        master.resizable(False, False)

    # this is the function called when the button is clicked
    def on_click_start(self):
        # self.send_command("start")
        if self.flocking_type.get() != "Select a flocking type":
            self.send_command(self.flocking_type.get())
            self.experiment_running = True
            self.start_ensure_sep_thread()
        else:
            messagebox.showinfo("Error", "Please select a flocking type")

    # this is the function called when the button is clicked
    def on_click_hold(self):
        self.send_command("hold")
        self.experiment_running = False

    def on_click_takeoff(self):
        self.send_command("takeoff")

    def on_click_land(self):
        self.send_command("land")

    def on_click_arm(self):
        self.send_command("arm")

    def on_click_return(self):
        self.alt_dict = gtools.create_swarm_dict(
            self.real_swarm_size, self.sitl_swarm_size
        )
        for key in self.alt_dict:
            self.alt_dict[key] = self.comms.swarm_telemetry[key].geodetic[2]
            print(self.comms.swarm_telemetry[key].geodetic[0])

        output_alt_dict = gtools.alt_calc(self.alt_dict, self.site_elevation)
        print(output_alt_dict)
        for key in output_alt_dict:
            self.comms.client.publish(key + "/home/altitude", str(output_alt_dict[key]))
        time.sleep(1)
        self.send_command("return")

    def on_click_launch(self):
        print("launch")
        if self.validate_int(self.sitl_drones_entry.get()) is True:
            self.sitl_swarm_size = int(self.sitl_drones_entry.get())
        else:
            tk.messagebox.showinfo("Error", "Please enter an Int")
        self.start_gazebo()
        self.start_mavsdk_servers()
        self.start_scripts()

    def on_click_select(self):
        print("select")
        firmware_path = filedialog.askdirectory()
        self.path_label.config(text=firmware_path)
        with open("config.txt", "w+") as f:
            f.write(firmware_path)

    def sitl_check(self):
        print("function activated")
        print(self.check_var.get())
        if self.check_var.get() == 0:
            self.sitl_frame.grid_remove()
        else:
            print("returning items")
            self.sitl_frame.grid()

    def on_click_relaunch_gazebo(self):
        if self.gazebo_process != None:
            sig = signal.SIGTERM
            os.killpg(os.getpgid(self.gazebo_process.pid), sig)

        self.start_gazebo()

    def on_click_relaunch_scripts(self):
        if self.script_process != None:
            sig = signal.SIGTERM
            for i in range(0, len(self.script_process)):
                os.killpg(os.getpgid(self.script_process[i].pid), sig)

        self.start_scripts()

    def on_click_confirm(self):
        # set new swarm size and then restart comms thread
        if self.validate_int(self.real_drones_entry.get()) and self.validate_int(
            self.sitl_drones_entry.get()
        ):
            self.initialise_swarm()
        else:
            tk.messagebox.showinfo("Error", "Please enter an Int")

        if self.comms_thread != None:
            self.comms.close()
        self.start_comms_thread()

        self.generate_status_elements()

    def initialise_swarm(self):
        self.real_swarm_size = int(self.real_drones_entry.get())
        self.sitl_swarm_size = int(self.sitl_drones_entry.get())
        self.swarm_size = self.real_swarm_size + self.sitl_swarm_size
        self.arm_status = gtools.create_swarm_dict(
            self.real_swarm_size, self.sitl_swarm_size
        )
        for key in self.arm_status.keys():
            self.arm_status[key] = False

    # Start the MQTT communication class in a new thread
    def start_comms_thread(self):
        self.comms = GroundCommunication(self.real_swarm_size, self.sitl_swarm_size)
        self.comms_thread = threading.Thread(
            target=asyncio.run, args=(self.comms.run_comms(),), daemon=True
        )
        self.comms_thread.start()
        callback_functions = {
            "arm_status": self.on_arm_status_update,
            "connection_status": self.on_connection_status_update,
        }

        # Bind the callbacks
        self.comms.bind_callback_functions(callback_functions)

    def start_gazebo(self):
        self.gazebo_process = subprocess.Popen(
            [
                "bash",
                "gazebo_sitl_multiple_run.sh",
                "-n",
                str(self.sitl_swarm_size),
                "-w",
                "hough_end",
            ],
            cwd=self.path_label.cget("text") + "/Tools/",
            stdin=None,
            stdout=None,
            stderr=None,
            preexec_fn=os.setsid,
        )  # last argument important to allow process to be killed

    def start_mavsdk_servers(self):
        self.mavsdk_process = [None] * self.sitl_swarm_size
        for i in range(0, self.sitl_swarm_size):
            self.mavsdk_process[i] = subprocess.Popen(
                [
                    "./mavsdk_server",
                    "-p",
                    str(50041 + i),
                    "udp://:" + str(14540 + i),
                ],
                cwd=os.path.split(mavsdk.__file__)[0] + "/bin",
                stdin=None,
                stdout=None,
                stderr=None,
                preexec_fn=os.setsid,
            )

    def start_scripts(self):
        self.script_process = [None] * self.sitl_swarm_size
        for i in range(0, self.sitl_swarm_size):
            self.script_process[i] = subprocess.Popen(
                [
                    sys.executable,
                    "onboard.py",
                ],
                cwd=os.getcwd(),
                preexec_fn=os.setsid,
            )

    def generate_status_elements(self):
        if self.status_element_frame != {}:
            for element in self.status_element_frame.values():
                element.destroy()

        # Build dictionaries of status elements
        self.status_element_frame = gtools.create_swarm_dict(
            self.real_swarm_size, self.sitl_swarm_size
        )
        self.status_button = gtools.create_swarm_dict(
            self.real_swarm_size, self.sitl_swarm_size
        )
        self.status_label = gtools.create_swarm_dict(
            self.real_swarm_size, self.sitl_swarm_size
        )

        print(self.status_element_frame)

        row = 0  # row iterator

        for key in self.status_element_frame.keys():
            self.status_element_frame[key] = tk.Frame(self.status_frame)
            self.status_element_frame[key].grid(row=row, column=0, sticky="ew")

            self.status_button[key] = tk.Button(
                self.status_element_frame[key],
                text=key,
                bg="#008B45",
                font=("arial", 12, "normal"),
            )
            self.status_button[key].grid(row=0, column=0, sticky="ew")

            self.status_label[key] = tk.Label(
                self.status_element_frame[key], text="STATUS"
            )
            self.status_label[key].grid(row=1, column=0, sticky="ew")
            row += 1

    # Callback triggered when arm status is updated
    def on_arm_status_update(self, agent, status):
        # messagebox.showinfo("Arm Status", status)
        if status == "True":
            self.arm_status[agent] = True
            self.status_label[agent].config(text="ARMED")
            self.change_button_colour(self.activated_button_colour)
            if all(self.arm_status.values()):
                self.set_site_elevation()
        else:
            self.arm_status[agent] = False
            self.status_label[agent].config(text="DISARMED")
            self.change_button_colour(self.normal_button_colour)

    def on_connection_status_update(self, agent, status):
        # if status == False:
        #     self.send_command("hold")
        #     self.experiment_running = False
        #     print(agent, " lost connection: holding")
        print(agent + "connection status: " + str(status))

    def change_button_colour(self, colour):
        self.takeoff_button.config(bg=colour)
        self.start_button.config(bg=colour)
        self.hold_button.config(bg=colour)
        self.land_button.config(bg=colour)
        self.return_button.config(bg=colour)

    def set_site_elevation(self):
        altitudes_dict = {}
        for key in self.comms.swarm_telemetry.keys():
            altitudes_dict[key] = self.comms.swarm_telemetry[key].geodetic[2]

        altitudes_list = list(altitudes_dict.values())
        self.site_elevation = min(altitudes_list)
        print(self.site_elevation)

    # sends mqtt commands to broker
    def send_command(self, command):
        print("send command called")
        self.comms.client.publish("commands", command)

    def validate_int(self, input):
        if input in "0123456789":
            try:
                int(input)
                return True
            except ValueError:
                return False
        else:
            return False

    def start_ensure_sep_thread(self):
        self.ensure_sep_thread = threading.Thread(
            target=asyncio.run, args=(self.ensure_seperation(),), daemon=True
        )
        self.ensure_sep_thread.start()

    async def ensure_seperation(self):
        print("task started")
        while self.experiment_running:
            proximity_list = gtools.proximity_check(
                self.comms.swarm_telemetry, min_proximity=10
            )

            if proximity_list:
                warning_string = ""
                for i in range(0, len(proximity_list)):
                    warning_string = (
                        warning_string
                        + proximity_list[i][0]
                        + " is "
                        + str(round(proximity_list[i][2], 2))
                        + "m from "
                        + proximity_list[i][1]
                    )
                    if i != len(proximity_list) - 1:
                        warning_string = warning_string + "\n"
                self.warning_frame.grid()
                self.warning_label.config(text=warning_string)
            else:
                self.warning_frame.grid_remove()

            await asyncio.sleep(1)

    def on_closing(self):
        if self.gazebo_process != None:
            sig = signal.SIGTERM
            os.killpg(os.getpgid(self.gazebo_process.pid), sig)
            for i in range(0, len(self.script_process)):
                os.killpg(os.getpgid(self.mavsdk_process[i].pid), sig)
                os.killpg(os.getpgid(self.script_process[i].pid), sig)
        root.destroy()


if __name__ == "__main__":

    root = tk.Tk()
    app = App(root)
    root.mainloop()
