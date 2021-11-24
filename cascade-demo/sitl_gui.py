#!/usr/bin/env python3

import sys
import subprocess
import mavsdk
import os
import signal
import tkinter as tk
from tkinter import filedialog
from tkinter import messagebox
import paho.mqtt.client as mqtt


class App:
    def __init__(self, master):
        self.master = master
        self.normal_button_colour = "#4E73ED"
        self.check_var = tk.IntVar(value=1)
        self.flocking_type = tk.StringVar(value="Select a flocking type")
        self.flocking_options = [
            "Simple Flocking",
            "Advanced Flocking",
            "Another type of Flocking",
        ]
        self.gazebo_process = None
        self.mavsdk_process = [None]
        self.script_process = [None]

        # This is the section of code which creates the main window
        master.rowconfigure(tuple(range(2)), minsize=100)
        master.grid_columnconfigure(tuple(range(3)), uniform="button", minsize=146)

        self.sitl_frame = tk.Frame(master)

        self.sitl_frame.columnconfigure(tuple(range(3)), weight=1)
        self.sitl_frame.grid(row=3, column=0, columnspan=3, sticky="ew")

        self.launch_btn = tk.Button(
            self.sitl_frame,
            text="Launch SITL Simulation",
            bg="#008B45",
            font=("arial", 12, "normal"),
            command=self.LaunchClickFunction,
        )
        self.launch_btn.grid(row=5, column=0, columnspan=3, sticky="e")

        self.select_firmware_btn = tk.Button(
            self.sitl_frame,
            text="Select",
            # bg="#008B45",
            font=("arial", 12, "normal"),
            command=self.SelectClickFunction,
        )
        self.select_firmware_btn.grid(row=4, column=2, sticky="e")

        tk.Button(
            master,
            text="Start",
            bg=self.normal_button_colour,
            font=("arial", 12, "normal"),
            command=self.StartClickFunction,
        ).grid(
            row=0,
            column=2,
            sticky="nesw",
        )

        tk.Button(
            master,
            text="Hold",
            bg=self.normal_button_colour,
            font=("arial", 12, "normal"),
            command=self.StopClickFunction,
        ).grid(row=1, column=0, sticky="nesw")

        tk.Button(
            master,
            text="Take Off",
            bg=self.normal_button_colour,
            font=("arial", 12, "normal"),
            command=self.TakeOffClickFunction,
        ).grid(row=0, column=1, sticky="nesw")

        tk.Button(
            master,
            text="Land",
            bg=self.normal_button_colour,
            font=("arial", 12, "normal"),
            command=self.LandClickFunction,
        ).grid(row=1, column=1, sticky="nesw")

        tk.Button(
            master,
            text="Arm",
            bg="#CD4F39",
            font=("arial", 12, "normal"),
            command=self.ArmClickFunction,
        ).grid(row=0, column=0, sticky="nesw")

        tk.Button(
            master,
            text="Return",
            bg=self.normal_button_colour,
            font=("arial", 12, "normal"),
            command=self.ReturnClickFunction,
        ).grid(row=1, column=2, sticky="nesw")

        self.flocking_menu = tk.OptionMenu(
            master, self.flocking_type, *self.flocking_options
        )
        self.flocking_menu.grid(row=2, column=0, columnspan=2, sticky="w")

        self.sitl_check = tk.Checkbutton(
            master,
            text="SITL",
            variable=self.check_var,
            onvalue=1,
            offvalue=0,
            command=self.SITLCheckFunction,
        )
        self.sitl_check.grid(row=2, column=2, sticky="e")

        self.no_drones_label = tk.Label(self.sitl_frame, text="Number of Drones:")
        self.no_drones_label.grid(row=3, column=0, sticky="w")

        self.firmware_label = tk.Label(self.sitl_frame, text="PX4 Firmware Path:")
        self.firmware_label.grid(row=4, column=0, sticky="w")

        self.path_label = tk.Label(self.sitl_frame)
        with open("config.txt") as f:
            self.firmware_path = f.read()
        self.path_label.config(text=self.firmware_path)
        self.path_label.grid(row=4, column=1, sticky="w")

        self.drone_no_entry = tk.Entry(self.sitl_frame)
        self.drone_no_entry.insert(tk.END, "3")
        self.drone_no_entry.grid(row=3, column=1, sticky="w")

        master.title("Cascade Demo")
        master.iconphoto(True, tk.PhotoImage(file="../img/cascade-logo.png"))
        master.protocol("WM_DELETE_WINDOW", self.on_closing)
        master.resizable(False, False)

    # this is the function called when the button is clicked
    def StartClickFunction(self):
        # self.send_command("start")
        if self.flocking_type.get() != "Select a flocking type":
            self.send_command(self.flocking_type.get())
        else:
            messagebox.showinfo("Error", "Please select a flocking type")

    # this is the function called when the button is clicked
    def StopClickFunction(self):
        self.send_command("hold")

    def TakeOffClickFunction(self):
        self.send_command("takeoff")

    def LandClickFunction(self):
        self.send_command("land")

    def ArmClickFunction(self):
        self.send_command("arm")

    def ReturnClickFunction(self):
        self.send_command("return")

    def LaunchClickFunction(self):
        print("launch")
        if self.validate_int(self.drone_no_entry.get()) is True:
            self.no_drones = int(self.drone_no_entry.get())
        else:
            tk.messagebox.showinfo("Error", "Please enter an Int")
        self.gazebo_process = subprocess.Popen(
            ["bash", "gazebo_sitl_multiple_run.sh", "-n", str(self.no_drones)],
            cwd=self.path_label.cget("text") + "/Tools/",
            stdin=None,
            stdout=None,
            stderr=None,
            preexec_fn=os.setsid,
        )  # last argument important to allow process to be killed

        self.mavsdk_process = [None] * self.no_drones
        self.script_process = [None] * self.no_drones

        for i in range(0, self.no_drones):
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

            self.script_process[i] = subprocess.Popen(
                [
                    sys.executable,
                    "onboard.py",
                    str(101 + i),
                    str(self.no_drones),
                    str(50041 + i),
                ],
                cwd=os.getcwd(),
                preexec_fn=os.setsid,
            )

    def SelectClickFunction(self):
        print("select")
        firmware_path = filedialog.askdirectory()
        self.path_label.config(text=firmware_path)
        with open("config.txt", "w+") as f:
            f.write(firmware_path)

    def SITLCheckFunction(self):
        print("function activated")
        print(self.check_var.get())
        if self.check_var.get() == 0:
            self.sitl_frame.grid_remove()
        else:
            print("returning items")
            self.sitl_frame.grid()

    # sends mqtt commands to broker
    def send_command(self, command):
        self.client = mqtt.Client()
        self.client.connect("localhost", 1883, 60)
        self.client.publish("commands", command)
        self.client.disconnect()

    def validate_int(self, input):
        if input in "0123456789":
            try:
                int(input)
                return True
            except ValueError:
                return False
        else:
            return False

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
