import paho.mqtt.client as mqtt


class PosVelNED:
    position = [0, 0, 0]
    velocity = [0, 0, 0]


# Create dict of PosVelNED objects with drone identifier e.g. P101 as the keys
drone_ids = range(101, 104)
drone_telemetry = {}
for i in drone_ids:
    drone_telemetry["P" + str(i)] = PosVelNED()


def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("+/telemetry/+")


def on_message_position(mosq, obj, msg):
    # Remove none numeric parts of string and then split into north east and down
    received_string = msg.payload.decode().strip("()")
    string_list = received_string.split(", ")
    position = [float(i) for i in string_list]
    drone_telemetry[msg.topic[0:4]].position = position


def on_message_velocity(mosq, obj, msg):
    # Remove none numeric parts of string and then split into north east and down
    received_string = msg.payload.decode().strip("[]")
    string_list = received_string.split(", ")
    velocity = [float(i) for i in string_list]
    drone_telemetry[msg.topic[0:4]].velocity = velocity


client = mqtt.Client()

client.message_callback_add("+/telemetry/position", on_message_position)
client.message_callback_add("+/telemetry/velocity", on_message_velocity)
client.connect("localhost", 1883, 60)  # change localhost to IP of broker
client.on_connect = on_connect

client.loop_forever()
