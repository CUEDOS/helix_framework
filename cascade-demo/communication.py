import asyncio
import paho.mqtt.client as mqtt


class PosVelNED:
    geodetic = [0, 0, 0]
    position_ned = [0, 0, 0]
    velocity_ned = [0, 0, 0]


class Communication:
    client = mqtt.Client()

    def __init__(self, swarm_size):
        self.swarm_size = swarm_size
        self.create_dict()

    def create_dict(self):
        self.drone_ids = range(101, 101 + self.swarm_size)
        self.swarm_pos_vel = {}
        for i in self.drone_ids:
            self.swarm_pos_vel["P" + str(i)] = PosVelNED()

    async def run_comms(self):
        Communication.client.message_callback_add(
            "+/telemetry/geodetic", self.on_message_geodetic
        )
        Communication.client.message_callback_add(
            "+/telemetry/position_ned", self.on_message_position
        )
        Communication.client.message_callback_add(
            "+/telemetry/velocity_ned", self.on_message_velocity
        )
        Communication.client.connect_async(
            "localhost", 1883, 60
        )  # change localhost to IP of broker
        Communication.client.on_connect = self.on_connect
        Communication.client.loop_start()

    # callback triggeed on connection to MQTT
    def on_connect(self, client, userdata, flags, rc):
        print("MQTT connected to broker with result code " + str(rc))
        client.subscribe("+/telemetry/+")

    def on_message_geodetic(self, mosq, obj, msg):
        # Remove none numeric parts of string and then split into north east and down
        received_string = msg.payload.decode().strip("()")
        string_list = received_string.split(", ")
        geodetic = [float(i) for i in string_list]
        # time.sleep(1)  # simulating comm latency
        self.swarm_pos_vel[msg.topic[0:4]].geodetic = geodetic

    def on_message_position(self, mosq, obj, msg):
        # Remove none numeric parts of string and then split into north east and down
        received_string = msg.payload.decode().strip("()")
        string_list = received_string.split(", ")
        position = [float(i) for i in string_list]
        # time.sleep(1)  # simulating comm latency
        self.swarm_pos_vel[msg.topic[0:4]].position_ned = position

    def on_message_velocity(self, mosq, obj, msg):
        # Remove none numeric parts of string and then split into north east and down
        received_string = msg.payload.decode().strip("[]")
        string_list = received_string.split(", ")
        velocity = [float(i) for i in string_list]
        # time.sleep(1)  # simulating comm latency
        self.swarm_pos_vel[msg.topic[0:4]].velocity_ned = velocity


# Inherits from Communication class, overriding methods specific to drones.
class DroneCommunication(Communication):
    def __init__(self, swarm_size, id):
        self.id = id
        self.swarm_size = swarm_size
        self.current_command = "none"
        # self.return_alt = 10
        self.create_dict()

    async def run_comms(self):
        Communication.client.message_callback_add(
            "+/telemetry/geodetic", self.on_message_geodetic
        )
        Communication.client.message_callback_add(
            "+/telemetry/position_ned", self.on_message_position
        )
        Communication.client.message_callback_add(
            "+/telemetry/velocity_ned", self.on_message_velocity
        )
        Communication.client.message_callback_add(
            self.id + "/home/altitude", self.on_message_home
        )
        Communication.client.message_callback_add("commands", self.on_message_command)
        Communication.client.connect_async(
            "localhost", 1883, 60
        )  # change localhost to IP of broker
        Communication.client.on_connect = self.on_connect
        Communication.client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        print("MQTT connected to broker with result code " + str(rc))
        client.subscribe("+/telemetry/+")
        client.subscribe("commands")
        client.subscribe("+/home/altitude")

    def on_message_command(self, mosq, obj, msg):
        print("received command")
        self.current_command = msg.payload.decode()

    def on_message_home(self, mosq, obj, msg):
        # agent.return_alt = msg.payload.decode()
        print("received new home altitude")
        self.return_alt = float(msg.payload.decode())
