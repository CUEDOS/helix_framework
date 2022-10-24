from asyncore import read
import paho.mqtt.client as mqtt
import asyncio
import time
from os import listdir
from os.path import isfile, join


class ExperimentRunner:
    def __init__(self, agents):
        self.client = mqtt.Client()
        self.broker_ip = "localhost"
        self.connected = False
        self.status = {}
        # for agent in agents:
        #     self.client.message_callback_add(
        #         agent.id + "/status", self.on_message_status
        #     )
        for agent in agents:
            self.status[agent] = "NONE"

    async def run_comms(self):
        self.client.message_callback_add("+/status", self.on_message_status)
        # set message to be sent when connection is lost

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
        self.client.subscribe("+/status")
        time.sleep(1)

    def on_disconnect(self, client, userdata, rc):
        print("disconnected from broker")
        self.connected = False

    def on_message_status(self, mosq, obj, msg):
        self.status[msg.topic.split("/")[0]] = msg.payload.decode()
        print(self.status)


async def command_experiments():
    await asyncio.sleep(5)
    for experiment in experiments:
        print("sending " + experiment)
        for agent in agents:
            experiment_runner.client.publish(
                agent + "/current_experiment",
                "closing_experiments/" + experiment,
            )
        await asyncio.sleep(2)

        print("sending pre start")
        ready_flag = False
        for agent in agents:
            experiment_runner.client.publish("commands/" + agent, "pre_start")
        while ready_flag == False:
            ready_flag = True
            for agent in agents:
                if experiment_runner.status[agent] != "READY":
                    ready_flag = False
            await asyncio.sleep(1)
        # while all(value != "READY" for value in experiment_runner.status.values()):
        #     time.sleep(1)

        await asyncio.sleep(5)

        print("pre start finished, sending start")

        for agent in agents:
            experiment_runner.client.publish("commands/" + agent, "Experiment")

        # while all(value != "DONE" for value in experiment_runner.status.values()):
        #     time.sleep(1)
        ready_flag = False
        while ready_flag == False:
            ready_flag = True
            for agent in agents:
                if experiment_runner.status[agent] != "DONE":
                    ready_flag = False
            await asyncio.sleep(1)

        for agent in agents:
            experiment_runner.client.publish("commands/" + agent, "hold")

        await asyncio.sleep(5)

        print(experiment + " finished")


if __name__ == "__main__":
    agents = ["S001", "S002"]
    experiment_folder_path = "experiments/closing_experiments"
    experiments = [
        f
        for f in listdir(experiment_folder_path)
        if isfile(join(experiment_folder_path, f))
    ]
    for i in range(len(experiments)):
        experiments[i] = experiments[i].split(".")[0]
        # remove .json as onboard doesnt expect it
    print(experiments)
    experiment_runner = ExperimentRunner(agents)
    asyncio.ensure_future(experiment_runner.run_comms())
    asyncio.ensure_future(command_experiments())
    event_loop = asyncio.get_event_loop()
    event_loop.run_forever()
