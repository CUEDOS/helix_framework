import tkinter as tk
from tkinter import ttk
from tkinter import *
import paho.mqtt.client as mqtt


# this is the function called when the button is clicked
def StartClickFunction():
    client.publish("commands", "start")
    print("clicked")


# this is the function called when the button is clicked
def StopClickFunction():
    client.publish("commands", "stop")
    print("clicked")


def TakeOffClickFunction():
    client.publish("commands", "takeoff")
    print("clicked")


def LandClickFunction():
    client.publish("commands", "land")
    print("clicked")


client = mqtt.Client()
client.connect("localhost", 1883, 60)

root = Tk()

# This is the section of code which creates the main window
root.geometry("636x525")
root.configure(background="#F0F8FF")
root.title("Swarm Experiment")


# This is the section of code which creates a button
Button(
    root,
    text="Start",
    bg="#008B45",
    font=("arial", 12, "normal"),
    command=StartClickFunction,
).place(x=71, y=57)


# This is the section of code which creates a button
Button(
    root,
    text="Stop",
    bg="#CD4F39",
    font=("arial", 12, "normal"),
    command=StopClickFunction,
).place(x=71, y=117)

Button(
    root,
    text="Take Off",
    bg="#CD4F39",
    font=("arial", 12, "normal"),
    command=TakeOffClickFunction,
).place(x=71, y=177)

Button(
    root,
    text="Land",
    bg="#CD4F39",
    font=("arial", 12, "normal"),
    command=LandClickFunction,
).place(x=71, y=237)


root.mainloop()
