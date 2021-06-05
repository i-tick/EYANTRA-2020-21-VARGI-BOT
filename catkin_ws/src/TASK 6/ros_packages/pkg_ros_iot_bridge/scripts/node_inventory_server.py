#!/usr/bin/env python

# Importing the required packages

import paho.mqtt.client as mqtt
import json
import requests

# MQTT dashboard URL and port code
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

    -----------

    client ,
    userdata ,
    message

    Returns

    -------

    NULL

    """

    json_mssg = json.loads(str(message.payload))
    print(json_mssg)

    # data to pushed to sheet
    parameters = {"id": "Inventory", "Team Id": "VB#0566", "Unique Id": "AiRiAkAk", "SKU": json_mssg['SKU'], "Item": json_mssg[
        'Item'], "Priority": json_mssg['Priority'], "Storage Number": json_mssg['Storage Number'], "Cost": json_mssg['Cost'], "Quantity": json_mssg['Quantity']}
    # our sheet
    URL = "https://script.google.com/macros/s/AKfycby_M9efwsF0Bc5DQc4X7CpIWALNYtvC6oyzcVUqiTSUaVdm7Y92KluR/exec"
    # eyantra sheet
    URL_eyantra = "https://script.google.com/macros/s/AKfycbw5xylppoda-8HPjt2Tzq4ShU_Xef-Ik-hEtBPcPk0gdGw8095j4RZ7/exec"
    response_eyantra = requests.get(URL_eyantra, params=parameters)
    response = requests.get(URL, params=parameters)
    print(response.content)


sub_client = mqtt.Client()
sub_client.on_connect = on_connect
sub_client.on_message = on_message
sub_client.connect(broker_url, broker_port)

sub_client.subscribe("/eyrc/vb/AiRiAkAk/inventory", qos=0)

sub_client.loop_forever()

print("Out of Loop. Exiting..")
