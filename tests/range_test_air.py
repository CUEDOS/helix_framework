import paho.mqtt.client as mqtt
import time

# This is the Publisher

client = mqtt.Client()
client.connect("localhost", 1883, 60)  # replace with IP of Broker
# client.publish("range_test", time.time());

while 1:
    loop_start_time = time.time()

    client.publish("range_test", time.time())

    # Checking frequency of the loop
    time.sleep(0.1 - (time.time() - loop_start_time))  # to make while 1 work at 10 Hz
    # print("loop duration=",(time.time()-loop_start_time))
