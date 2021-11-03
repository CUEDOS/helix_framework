import paho.mqtt.client as mqtt
import time
import csv

# This is the Subscriber

def on_connect(client, userdata, flags, rc):
  print("Connected with result code "+str(rc))
  client.subscribe("range_test")

def on_message(client, userdata, msg):
    message = msg.payload.decode().split(",")
    time_diff = time.time()-float(message[0])
    #csv_line = str(time_diff)+","+message[1]+","+message[2]+","+message[3]
    message[0] = str(time_diff)
    print(message)
    #print(csv_line)
    with open("range_test_data.csv", "a", encoding="UTF8") as f:
        writer = csv.writer(f)
        writer.writerow(message)
    
client = mqtt.Client()
client.connect("localhost",1883,60)

client.on_connect = on_connect
client.on_message = on_message

client.loop_forever()