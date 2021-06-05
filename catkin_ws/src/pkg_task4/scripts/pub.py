#!/usr/bin/env python

import paho.mqtt.client as mqtt
import time
import json

broker_url = "broker.mqttdashboard.com"
broker_port = 1883
pub_message = {"message": "hello"}
pub_topic = "/eyrc/vb/AitikDan/orders"

def on_publish(client, userdata, mid):
    print("--- Publisher ---")
    print("[INFO] Topic: {}".format(pub_topic))
    print("[INFO] Message Published: {}".format(pub_message))
    print("------------")

pub_client = mqtt.Client()
pub_client.on_publish = on_publish
pub_client.connect(broker_url, broker_port)
pub_message = json.dumps(pub_message)
while True:
    # pub_message = time.asctime( time.localtime(time.time()) )
    print(pub_message)
    pub_client.publish(topic=pub_topic, payload=pub_message, qos=0, retain=False)
    time.sleep(1)

print("Out of Loop. Exiting..")
