import asyncio
import time
import paho.mqtt.client as mqtt
import gtools
from data_structures import AgentTelemetry

CONST_BROKER_ADDRESS = "localhost"
# CONST_BROKER_ADDRESS = "broker.hivemq.com"


class Communication:
    client = mqtt.Client()

    def __init__(self, real_swarm_size, sitl_swarm_size):
        self.create_dict(real_swarm_size, sitl_swarm_size)
        self.connected = False

    def create_dict(self, real_swarm_size, sitl_swarm_size):
        self.swarm_telemetry = gtools.create_swarm_dict(
            real_swarm_size, sitl_swarm_size
        )
        for key in self.swarm_telemetry.keys():
            self.swarm_telemetry[key] = AgentTelemetry()

    async def run_comms(self):
        self.client.message_callback_add(
            "+/telemetry/geodetic", self.on_message_geodetic
        )
        self.client.message_callback_add(
            "+/telemetry/position_ned", self.on_message_position
        )
        self.client.message_callback_add(
            "+/telemetry/velocity_ned", self.on_message_velocity
        )
        self.client.connect_async(
            CONST_BROKER_ADDRESS, 1883, 60
        )  # change localhost to IP of broker
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.loop_start()

    def close(self):
        self.client.disconnect()
        self.client.loop_stop()

    # callback triggeed on connection to MQTT
    def on_connect(self, client, userdata, flags, rc):
        print("MQTT connected to broker with result code " + str(rc))
        self.connected = True
        client.subscribe("+/telemetry/+")
        client.subscribe("+/connection_status")

    def on_disconnect(self, client, userdata, rc):
        self.connected = False

    def on_message_geodetic(self, mosq, obj, msg):
        # Remove none numeric parts of string and then split into north east and down
        received_string = msg.payload.decode().strip("()")
        string_list = received_string.split(", ")
        geodetic = [float(i) for i in string_list]
        # time.sleep(1)  # simulating comm latency
        # replace reference to first 4 characters of topic with splitting topic at /
        self.swarm_telemetry[msg.topic[0:4]].geodetic = geodetic

    def on_message_position(self, mosq, obj, msg):
        # Remove none numeric parts of string and then split into north east and down
        received_string = msg.payload.decode().strip("()")
        string_list = received_string.split(", ")
        position = [float(i) for i in string_list]
        # time.sleep(1)  # simulating comm latency
        self.swarm_telemetry[msg.topic[0:4]].position_ned = position

    def on_message_velocity(self, mosq, obj, msg):
        # Remove none numeric parts of string and then split into north east and down
        received_string = msg.payload.decode().strip("[]")
        string_list = received_string.split(", ")
        velocity = [float(i) for i in string_list]
        # time.sleep(1)  # simulating comm latency
        self.swarm_telemetry[msg.topic[0:4]].velocity_ned = velocity

    # def bind_callback(self, callback):
    #     self.observers.append(callback)

    # def activate_callback(self, agent, data):
    #     for callback in self.observers:
    #         callback(agent, data)


# Inherits from Communication class, overriding methods specific to drones.
class DroneCommunication(Communication):
    def __init__(self, real_swarm_size, sitl_swarm_size, id, experiment):
        self.experiment = experiment
        self.first_connection = True
        self.connected = False
        self.id = id
        self.command_functions = {}
        self.current_command = "none"
        # self.return_alt = 10
        self.create_dict(real_swarm_size, sitl_swarm_size)

    async def run_comms(self):
        self.client.message_callback_add(
            "+/telemetry/geodetic", self.on_message_geodetic
        )
        self.client.message_callback_add(
            "+/telemetry/position_ned", self.on_message_position
        )
        self.client.message_callback_add(
            "+/telemetry/velocity_ned", self.on_message_velocity
        )
        self.client.message_callback_add(
            self.id + "/home/altitude", self.on_message_home
        )
        self.client.message_callback_add(
            self.id + "/corridor_points", self.on_message_corridor
        )
        self.client.message_callback_add("commands/" + self.id, self.on_message_command)
        # set message to be sent when connection is lost
        self.client.will_set(
            self.id + "/connection_status", "Disconnected", qos=2, retain=True
        )
        self.client.connect_async(
            CONST_BROKER_ADDRESS, 1883, keepalive=5
        )  # change localhost to IP of broker
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        self.connected = True
        print("MQTT connected to broker with result code " + str(rc))
        client.subscribe("+/telemetry/+")
        client.subscribe("commands/" + self.id)
        client.subscribe("+/home/altitude")
        client.subscribe("+/corridor_points")
        if self.first_connection:
            client.publish(
                "detection",
                self.id,
            )
            self.first_connection = False
        time.sleep(1)
        client.publish(self.id + "/connection_status", "Connected", qos=2, retain=True)

    def on_disconnect(self, client, userdata, rc):
        print("disconnected from broker")
        # client.publish(self.id + "/connection_status", "Disconnected", retain=True)
        self.connected = False
        self.activate_callback("disconnect")

    def on_message_command(self, mosq, obj, msg):
        print("received command")
        self.current_command = msg.payload.decode()
        self.activate_callback(msg.payload.decode())

    def on_message_home(self, mosq, obj, msg):
        # agent.return_alt = msg.payload.decode()
        print("received new home altitude")
        self.return_alt = float(msg.payload.decode())

    def on_message_corridor(self, mosq, obj, msg):
        # agent.return_alt = msg.payload.decode()
        print("received new corridor")
        self.experiment.set_corridor(msg.payload.decode())
        # self.return_alt = float(msg.payload.decode())

    def bind_command_functions(self, command_functions, event_loop):
        self.command_functions = command_functions
        self.event_loop = event_loop

    def activate_callback(self, command):
        print("activating callback")
        asyncio.ensure_future(self.command_functions[command](), loop=self.event_loop)


class GroundCommunication(Communication):
    def __init__(self, real_swarm_size, sitl_swarm_size):
        self.connected = False
        self.observers = []
        self.create_dict(real_swarm_size, sitl_swarm_size)

    async def run_comms(self):
        self.client.message_callback_add(
            "+/telemetry/geodetic", self.on_message_geodetic
        )
        self.client.message_callback_add(
            "+/telemetry/position_ned", self.on_message_position
        )
        self.client.message_callback_add(
            "+/telemetry/velocity_ned", self.on_message_velocity
        )
        self.client.message_callback_add(
            "+/telemetry/arm_status", self.on_message_arm_status
        )
        self.client.message_callback_add(
            "+/connection_status", self.on_message_connection_status
        )
        self.client.connect_async(
            CONST_BROKER_ADDRESS, 1883, 60
        )  # change localhost to IP of broker
        self.client.on_connect = self.on_connect
        self.client.loop_start()

    def on_message_connection_status(self, mosq, obj, msg):
        print("connection status updated")
        agent = msg.topic[0:4]
        if msg.payload.decode() == "0":
            print(agent, " lost connection")
            self.activate_callback("connection_status", agent, False)
        else:
            self.activate_callback("connection_status", agent, True)

    def on_message_arm_status(self, mosq, obj, msg):
        agent = msg.topic[0:4]
        self.swarm_telemetry[agent].arm_status = msg.payload.decode()
        self.activate_callback(
            "arm_status", agent, self.swarm_telemetry[agent].arm_status
        )

    def bind_callback_functions(self, callback_functions):
        self.callback_functions = callback_functions

    def activate_callback(self, callback, agent, status):
        self.callback_functions[callback](agent, status)
