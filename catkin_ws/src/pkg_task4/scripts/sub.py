#!/usr/bin/env python

import paho.mqtt.client as mqtt
import json
import requests

broker_url = "broker.mqttdashboard.com"
broker_port = 1883


def on_connect(client, userdata, flags, rc):
    print("[INFO] Connected With Result Code: " + str(rc))

def on_message(client, userdata, message):
    #print("--- Subscriber ---")
    #print("[INFO] Topic: {}".format(message.topic) )
    #print("[INFO] Message Recieved: {}".format(message.payload.decode()))
    json_mssg = json.loads(str(message.payload))
    print(json_mssg['city'])
    if json_mssg['item'] == 'Clothes':
        Priority = 'LP'
        Cost = 150
    elif json_mssg['item'] == 'Food':
        Priority = 'MP'
        Cost = 250
    elif json_mssg['item'] == 'Medicine':
        Priority = 'HP'
        Cost = 450
    parameters = {"id":"IncomingOrders","Team Id": "VB#0566","Unique Id":"AiRiAkAk","Order Id":json_mssg["order_id"],"Order Date and Time":json_mssg["order_time"],"Item":json_mssg["item"],"Priority":Priority,"Order Quantity":json_mssg["qty"],"City":json_mssg["city"],"Longitude":json_mssg["lon"],"Latitude":json_mssg["lat"],"Cost":Cost}
    URL = "https://script.google.com/macros/s/AKfycby_M9efwsF0Bc5DQc4X7CpIWALNYtvC6oyzcVUqiTSUaVdm7Y92KluR/exec"
    response = requests.get(URL, params=parameters)
    print(response.content)

    parameters = {"id":"Dashboard","Team Id":"VB#0566","Unique Id":"AiRiAkAk","Order Id":json_mssg["order_id"],"Item":json_mssg["item"],"Priority":Priority,"Quantity":1,"City":json_mssg["city"],"Longitude":json_mssg["lon"],"Latitude":json_mssg["lat"],"Order Dispatched":"NO","Order Shipped":"NO","Order Time":json_mssg["order_time"]}
    URL2 = "https://script.google.com/macros/s/AKfycbzXwIg573leL8Oy2aFIieRgqROp8Wfn1mT-_Ue864BuaZmVCvLgpb3w6A/exec"
    response = requests.get(URL2,params = parameters)
    print(response.content)
    print("------------")

sub_client = mqtt.Client()
sub_client.on_connect = on_connect
sub_client.on_message = on_message
sub_client.connect(broker_url, broker_port)

sub_client.subscribe("/eyrc/vb/AitikDan/orders", qos=0)

sub_client.loop_forever()

print("Out of Loop. Exiting..")