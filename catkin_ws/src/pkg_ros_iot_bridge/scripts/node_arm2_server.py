#!/usr/bin/env python

# Importing the required libraries
import paho.mqtt.client as mqtt
import json
import requests

# MQTT dashboard URl and port code
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
    print(json_mssg)
    # parameters
    parameters_1 = {"id": "OrdersShipped", "Team Id": "VB#0566", "Unique Id": "AiRiAkAk", "Order ID": json_mssg["Order Id"], "City": json_mssg["City"], "Item": json_mssg["Item"], "Priority": json_mssg[
        "Priority"], "Shipped Quantity": 1, "Cost": json_mssg["Cost"], "Shipped Status": json_mssg["Order Shipped"], "Shipped Date and Time": json_mssg["Shipping Time"], "Estimated Time of Delivery": json_mssg["Estimated Time of Delivery"]}
    parameters_2 = json_mssg
    # our sheet
    URL_shipped = "https://script.google.com/macros/s/AKfycbxEujrxYzn6uhcI9IKyHt-PumJxPJFYRxYtC3TRhW1fHgQ5yiFIjHIulg/exec"
    # eyantra sheet
    URL_eyantra = "https://script.google.com/macros/s/AKfycbw5xylppoda-8HPjt2Tzq4ShU_Xef-Ik-hEtBPcPk0gdGw8095j4RZ7/exec"
    # our dashboard
    URL_dashboard = "https://script.google.com/macros/s/AKfycbzXwIg573leL8Oy2aFIieRgqROp8Wfn1mT-_Ue864BuaZmVCvLgpb3w6A/exec"
    response = requests.get(URL_shipped, params=parameters_1)
    response_eyantra = requests.get(URL_eyantra, params=parameters_1)
    print(response.content)
    response_1 = requests.get(URL_dashboard, params=parameters_2)
    print(response_1.content)


sub_client = mqtt.Client()
sub_client.on_connect = on_connect
sub_client.on_message = on_message
sub_client.connect(broker_url, broker_port)

sub_client.subscribe("/eyrc/vb/AiRiAkAk/ur5_2", qos=0)

sub_client.loop_forever()

print("Out of Loop. Exiting..")
