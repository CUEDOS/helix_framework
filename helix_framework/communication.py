import asyncio
import time
import paho.mqtt.client as mqtt
import gtools
from data_structures import AgentTelemetry
import struct


class DroneCommunication:
    client = mqtt.Client()

    def __init__(self, agent, swarm_manager):
        self.agent = agent
        self.swarm_manager = swarm_manager
        self.broker_ip = agent.broker_ip
        self.first_connection = True
        self.connected = False
        self.id = agent.id
        self.command_functions = {}
        self.current_command = "none"

    async def run_comms(self):
        # self.client.message_callback_add(
        #     "+/telemetry/geodetic", self.on_message_geodetic
        # )
        # self.client.message_callback_add(
        #     "+/telemetry/position_ned", self.on_message_position
        # )
        # self.client.message_callback_add(
        #     "+/telemetry/velocity_ned", self.on_message_velocity
        # )
        self.client.message_callback_add("+/T", self.on_message_telemetry)
        self.client.message_callback_add(
            self.id + "/home/altitude", self.on_message_home
        )
        self.client.message_callback_add(
            self.id + "/corridor_points", self.on_message_corridor
        )
        self.client.message_callback_add(
            self.id + "/update_parameters", self.on_message_update_parameters
        )
        self.client.message_callback_add(
            self.id + "/current_experiment", self.on_message_current_experiment
        )
        self.client.message_callback_add("detection", self.on_message_detection)
        self.client.message_callback_add("commands/" + self.id, self.on_message_command)
        self.client.message_callback_add("emergency_stop", self.on_message_stop)
        # set message to be sent when connection is lost
        self.client.will_set(
            self.id + "/connection_status", "Disconnected", qos=2, retain=True
        )
        self.client.connect_async(
            self.broker_ip, 1883, keepalive=5
        )  # change localhost to IP of broker
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.loop_start()

    def close(self):
        self.client.disconnect()
        self.client.loop_stop()

    def on_connect(self, client, userdata, flags, rc):
        self.connected = True
        print("MQTT connected to broker with result code " + str(rc))
        client.subscribe("detection")
        client.subscribe("emergency_stop")
        client.subscribe("commands/" + self.id)
        client.subscribe("+/home/altitude")
        client.subscribe("+/corridor_points")
        client.subscribe(self.id + "/update_parameters")
        client.subscribe(self.id + "/current_experiment")
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

    def on_message_detection(self, mosq, obj, msg):
        self.add_agent(msg.payload.decode())

    def on_message_stop(self, mosq, obj, msg):
        print("STOPPING")
        self.current_command = "hold"
        self.activate_callback("hold")

    def on_message_command(self, mosq, obj, msg):
        print("received command")
        self.current_command = msg.payload.decode()
        self.activate_callback(msg.payload.decode())

    def on_message_home(self, mosq, obj, msg):
        # agent.return_alt = msg.payload.decode()
        print("received new home altitude")
        self.return_alt = float(msg.payload.decode())

    def on_message_corridor(self, mosq, obj, msg):
        print("received new corridor")
        # self.experiment.set_corridor(msg.payload.decode())

    def on_message_current_experiment(self, mosq, obj, msg):
        print("selecting experiment")
        self.agent.current_experiment = msg.payload.decode()

    def on_message_telemetry(self, mosq, obj, msg):
        unpacked_bytes = struct.unpack("10f", msg.payload)
        geodetic = unpacked_bytes[0:3]
        position_ned = unpacked_bytes[3:6]
        velocity_ned = unpacked_bytes[6:9]
        heading = unpacked_bytes[9]
        # replace reference to first 4 characters of topic with splitting topic at /
        self.swarm_manager.telemetry[msg.topic[0:4]].geodetic = list(geodetic)
        self.swarm_manager.telemetry[msg.topic[0:4]].position_ned = list(position_ned)
        self.swarm_manager.telemetry[msg.topic[0:4]].velocity_ned = list(velocity_ned)
        self.swarm_manager.telemetry[msg.topic[0:4]].heading = heading

    # def on_message_geodetic(self, mosq, obj, msg):
    #     # Remove none numeric parts of string and then split into north east and down
    #     received_string = msg.payload.decode().strip("()")
    #     string_list = received_string.split(", ")
    #     geodetic = [float(i) for i in string_list]
    #     # time.sleep(1)  # simulating comm latency
    #     # replace reference to first 4 characters of topic with splitting topic at /
    #     self.swarm_manager.telemetry[msg.topic[0:4]].geodetic = geodetic

    # def on_message_position(self, mosq, obj, msg):
    #     # Remove none numeric parts of string and then split into north east and down
    #     received_string = msg.payload.decode().strip("()")
    #     string_list = received_string.split(", ")
    #     position = [float(i) for i in string_list]
    #     # time.sleep(1)  # simulating comm latency
    #     self.swarm_manager.telemetry[msg.topic[0:4]].position_ned = position

    # def on_message_velocity(self, mosq, obj, msg):
    #     # Remove none numeric parts of string and then split into north east and down
    #     received_string = msg.payload.decode().strip("[]")
    #     string_list = received_string.split(", ")
    #     velocity = [float(i) for i in string_list]
    #     # time.sleep(1)  # simulating comm latency
    #     self.swarm_manager.telemetry[msg.topic[0:4]].velocity_ned = velocity

    def on_message_update_parameters(self, mosq, obj, msg):
        print("received updated parameter")
        self.agent.update_parameter(msg.payload.decode())

    def bind_command_functions(self, command_functions, event_loop):
        self.command_functions = command_functions
        self.event_loop = event_loop

    def activate_callback(self, command):
        print("activating callback")
        asyncio.ensure_future(self.command_functions[command](), loop=self.event_loop)

    def add_agent(self, new_id):
        # adds a new agent to the swarm if they are not already present
        if new_id not in self.swarm_manager.telemetry:
            self.swarm_manager.telemetry[new_id] = AgentTelemetry()
            self.client.subscribe(new_id + "/telemetry/+")
            # publish ID so that new agent can add it to their dict
            self.client.publish(
                "detection",
                self.id,
            )
