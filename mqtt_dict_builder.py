import paho.mqtt.client as mqtt

no_drones = range(1,4)
defaults = {'qx': 0, 'qy': 0,'qz': 0}
telemetry = dict.fromkeys(no_drones, defaults)

def on_connect(client, userdata, flags, rc):
  print("Connected with result code "+str(rc))
  client.subscribe("+/telemetry/position/#")

def on_message_qx(mosq, obj, msg):
    telemetry[int(msg.topic[0])]["qx"] = msg.payload
    print(telemetry)


def on_message_qy(mosq, obj, msg):
    telemetry[int(msg.topic[0])]["qy"] = msg.payload
    print(telemetry)


def on_message_qz(mosq, obj, msg):
    telemetry[int(msg.topic[0])]["qz"] = msg.payload
    print(telemetry)


client = mqtt.Client()

# Add message callbacks that will only trigger on a specific subscription match.
client.message_callback_add("+/telemetry/position/qx", on_message_qx)
client.message_callback_add("+/telemetry/position/qy", on_message_qy)
client.message_callback_add("+/telemetry/position/qz", on_message_qz)
client.connect("localhost",1883,60) #change localhost to IP of broker
client.on_connect = on_connect

client.loop_forever()