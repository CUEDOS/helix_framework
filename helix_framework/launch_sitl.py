import sys
import subprocess
from turtle import onclick
import mavsdk
import os
import signal
import time


def signal_handler(sig, frame):
    on_closing()
    sys.exit(0)


def start_gazebo():
    gazebo_process = subprocess.Popen(
        [
            "bash",
            "gazebo_sitl_multiple_run.sh",
            "-n",
            str(sitl_swarm_size),
            "-w",
            "llanbedr",
        ],
        cwd=firmware_path + "/Tools/",
        stdin=None,
        stdout=None,
        stderr=None,
        preexec_fn=os.setsid,
    )  # last argument important to allow process to be killed
    return gazebo_process


def start_mavsdk_servers():
    mavsdk_process = [None] * sitl_swarm_size
    for i in range(0, sitl_swarm_size):
        mavsdk_process[i] = subprocess.Popen(
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
    return mavsdk_process


def start_scripts():
    script_process = [None] * sitl_swarm_size
    for i in range(0, sitl_swarm_size):
        script_process[i] = subprocess.Popen(
            [
                sys.executable,
                "onboard.py",
                "SITL_Parameters/S" + str(i + 1).zfill(3) + "_parameters.json",
            ],
            cwd=os.getcwd(),
            preexec_fn=os.setsid,
        )
        time.sleep(1)
    return script_process


def on_closing():
    if gazebo_process != None:
        sig = signal.SIGTERM
        os.killpg(os.getpgid(gazebo_process.pid), sig)
        for i in range(0, len(script_process)):
            os.killpg(os.getpgid(mavsdk_process[i].pid), sig)
            os.killpg(os.getpgid(script_process[i].pid), sig)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    print("Press Ctrl+C to close")
    sitl_swarm_size = int(sys.argv[1])
    # firmware_path = "/home/m74744sa/PX4-Autopilot"
    firmware_path = "/home/m74744sa/PX4-Autopilot"
    gazebo_process = start_gazebo()
    mavsdk_process = start_mavsdk_servers()
    time.sleep(20)
    script_process = start_scripts()

    signal.pause()
