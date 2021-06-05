#!/usr/bin/env python

# Importing the required packages
import paho.mqtt.client as mqtt
import json
import requests
import rospy

# MQTT Dashboard Url and port code

broker_url = "broker.mqttdashboard.com"
broker_port = 1883


def on_connect(client, userdata, flags, rc):
    """
    Used to print the connection info on the terminal along with rc code

    Parameters

    ----------

    client ,
    userdata ,
    flags ,
    rc

    Returns

    -------

    NULL

    """
    print("[INFO] Connected With Result Code: " + str(rc))


def on_message(client, userdata, message):
    """
    Contains the userdata and message received

    Pushes data into google spreaasheets for the ur5 2 arm

    Parameters

    ----------

    client ,
    userdata ,
    message

    Returns

    -------

    NULL

    """

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

    # data to pushed
    print(rospy.get_time())
    parameters = {"id": "IncomingOrders", "Team Id": "VB#0566", "Unique Id": "AiRiAkAk", "Order ID": json_mssg["order_id"], "Order Date and Time": json_mssg["order_time"], "Item": json_mssg[
        "item"], "Priority": Priority, "Order Quantity": json_mssg["qty"], "City": json_mssg["city"], "Longitude": json_mssg["lon"], "Latitude": json_mssg["lat"], "Cost": Cost}
    # our sheet
    URL = "https://script.google.com/macros/s/AKfycby_M9efwsF0Bc5DQc4X7CpIWALNYtvC6oyzcVUqiTSUaVdm7Y92KluR/exec"
    # eyantra sheet
    URL_eyantra = "https://script.google.com/macros/s/AKfycbw5xylppoda-8HPjt2Tzq4ShU_Xef-Ik-hEtBPcPk0gdGw8095j4RZ7/exec"
    res = requests.get(URL_eyantra, params=parameters)
    response = requests.get(URL, params=parameters)
    print(response.content)

    parameters = {"id": "Dashboard", "Team Id": "VB#0566", "Unique Id": "AiRiAkAk", "Order Id": json_mssg["order_id"], "Item": json_mssg["item"], "Priority": Priority, "Quantity": 1, "City": json_mssg[
        "city"], "Longitude": json_mssg["lon"], "Latitude": json_mssg["lat"], "Order Dispatched": "NO", "Order Shipped": "NO", "Order Time": json_mssg["order_time"],"Incoming Sim Time": rospy.get_time()}
    # our dashboard
    URL2 = "https://script.google.com/macros/s/AKfycbzXwIg573leL8Oy2aFIieRgqROp8Wfn1mT-_Ue864BuaZmVCvLgpb3w6A/exec"
    response = requests.get(URL2, params=parameters)
    print(response.content)
    print("------------")


rospy.init_node('node_incomingOrders_sheet', anonymous=True)
sub_client = mqtt.Client()
sub_client.on_connect = on_connect
sub_client.on_message = on_message
sub_client.connect(broker_url, broker_port)

sub_client.subscribe("/eyrc/vb/AiRiAkAk/orders", qos=0)

sub_client.loop_forever()

print("Out of Loop. Exiting..")
