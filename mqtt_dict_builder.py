import paho.mqtt.client as mqtt
from struct import *

no_drones = range(0,3)
defaults = {'qx': 0, 'qy': 0, 'qz': 0}
telemetry = []
for i in no_drones:
    telemetry.append(dict(defaults))

def on_connect(client, userdata, flags, rc):
  print("Connected with result code "+str(rc))
  client.subscribe("+/telemetry/position/#")

def on_message(mosq, obj, msg):
    telemetry[int(msg.topic[0])-1][msg.topic.partition("position/")[2]] = float(msg.payload)
    print(telemetry)

client = mqtt.Client()

client.message_callback_add("+/telemetry/position/+", on_message)
client.connect("localhost",1883,60) #change localhost to IP of broker
client.on_connect = on_connect

client.loop_forever()