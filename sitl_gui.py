#!/home/r32401vc/anaconda3/bin/python3

import sys
import subprocess
import mavsdk
import os
import signal
import tkinter as tk
from tkinter import ttk
from tkinter import filedialog
from tkinter import messagebox
from tkinter import *
import paho.mqtt.client as mqtt

# Global Variables
gazebo = None

# this is the function called when the button is clicked
def StartClickFunction():
    send_command("start")


# this is the function called when the button is clicked
def StopClickFunction():
    send_command("stop")


def TakeOffClickFunction():
    send_command("takeoff")


def LandClickFunction():
    send_command("land")


def LaunchClickFunction():
    print("launch")
    if validate_int(drone_no_entry.get()) is True:
        no_drones = int(drone_no_entry.get())
    else:
        messagebox.showinfo("Error", "Please enter an Int")
    global gazebo
    gazebo = subprocess.Popen(
        ["bash", "gazebo_sitl_multiple_run.sh", "-n", str(no_drones)],
        cwd=path_label.cget("text") + "/Tools/",
        stdin=None,
        stdout=None,
        stderr=None,
        preexec_fn=os.setsid,
    )  # last argument important to allow process to be killed

    global mavsdk_server
    mavsdk_server = [None] * no_drones
    global script
    script = [None] * no_drones

    for i in range(0, no_drones):
        mavsdk_server[i] = subprocess.Popen(
            [
                "./mavsdk_server",
                "gazebo_sitl_multiple_run.sh",
                "-p",
                str(50041 + i),
                "udp://:" + str(14540 + i),
            ],
            cwd=os.path.split(mavsdk.__file__)[0] + "/bin",
            stdin=None,
            stdout=None,
            stderr=None,
        )

        script[i] = subprocess.Popen(
            [
                sys.executable,
                "simple_flocking.py",
                str(101 + i),
                str(no_drones),
                str(50041 + i),
            ],
            cwd=os.getcwd(),
        )


def SelectClickFunction():
    print("select")
    firmware_path = filedialog.askdirectory()
    path_label.config(text=firmware_path)
    with open("config.txt", "w") as f:
        f.write(firmware_path)


def SITLCheckFunction():
    print("function activated")
    print(check_var.get())
    if check_var.get() == 0:
        # drone_no_entry.grid_remove()
        # firmware_label.grid_remove()
        # launch_btn.grid_remove()
        # select_firmware_btn.grid_remove()
        # path_label.grid_remove()
        # no_drones_label.grid_remove()
        sitl_frame.grid_remove()
    else:
        print("returning items")
        # drone_no_entry.grid()
        # firmware_label.grid()
        # launch_btn.grid()
        # select_firmware_btn.grid()
        # path_label.grid()
        # no_drones_label.grid()
        sitl_frame.grid()


# sends mqtt commands to broker
def send_command(command):
    client = mqtt.Client()
    client.connect("localhost", 1883, 60)
    client.publish("commands", command)
    client.disconnect()


def validate_int(input):
    if input in "0123456789":
        try:
            int(input)
            return True
        except ValueError:
            return False
    else:
        return False


def on_closing():
    if gazebo != None:
        sig = signal.SIGTERM
        os.killpg(os.getpgid(gazebo.pid), sig)
        for i in range(0, len(script)):
            os.kill(mavsdk_server[i].pid, sig)
            os.kill(script[i].pid, sig)
    root.destroy()


root = Tk()
check_var = IntVar(value=1)
# This is the section of code which creates the main window
root.rowconfigure(tuple(range(2)), minsize=100)
# root.columnconfigure(tuple(range(2)), uniform="button", minsize=210)
root.grid_columnconfigure(tuple(range(2)), uniform="button", minsize=220)

sitl_frame = Frame(root)

sitl_frame.columnconfigure(tuple(range(3)), weight=1)
sitl_frame.grid(row=3, column=0, columnspan=2, sticky="ew")

launch_btn = Button(
    sitl_frame,
    text="Launch SITL Simulation",
    bg="#008B45",
    font=("arial", 12, "normal"),
    command=LaunchClickFunction,
)
launch_btn.grid(row=5, column=0, columnspan=3, sticky="e")

select_firmware_btn = Button(
    sitl_frame,
    text="Select",
    # bg="#008B45",
    font=("arial", 12, "normal"),
    command=SelectClickFunction,
)
select_firmware_btn.grid(row=4, column=2, sticky="e")

Button(
    root,
    text="Start",
    bg="#008B45",
    font=("arial", 12, "normal"),
    command=StartClickFunction,
).grid(
    row=1,
    column=0,
    sticky="nesw",
)

Button(
    root,
    text="Hold",
    bg="#CD4F39",
    font=("arial", 12, "normal"),
    command=StopClickFunction,
).grid(row=0, column=1, sticky="nesw")

Button(
    root,
    text="Take Off",
    bg="#CD4F39",
    font=("arial", 12, "normal"),
    command=TakeOffClickFunction,
).grid(row=0, column=0, sticky="nesw")

Button(
    root,
    text="Land",
    bg="#CD4F39",
    font=("arial", 12, "normal"),
    command=LandClickFunction,
).grid(row=1, column=1, sticky="nesw")

sitl_check = Checkbutton(
    root,
    text="SITL",
    variable=check_var,
    onvalue=1,
    offvalue=0,
    command=SITLCheckFunction,
)
sitl_check.grid(row=2, column=0, columnspan=2, sticky="w")

no_drones_label = Label(sitl_frame, text="Number of Drones:")
no_drones_label.grid(row=3, column=0, sticky="w")

firmware_label = Label(sitl_frame, text="PX4 Firmware Path:")
firmware_label.grid(row=4, column=0, sticky="w")

path_label = Label(sitl_frame)
with open("config.txt") as f:
    firmware_path = f.read()
path_label.config(text=firmware_path)
path_label.grid(row=4, column=1, sticky="w")

drone_no_entry = Entry(sitl_frame)
drone_no_entry.insert(END, "3")
drone_no_entry.grid(row=3, column=1, sticky="w")

root.title("Cascade Demo")
root.iconphoto(True, tk.PhotoImage(file="img/cascade-logo.png"))
root.protocol("WM_DELETE_WINDOW", on_closing)
# root.resizable(False, False)
root.mainloop()
