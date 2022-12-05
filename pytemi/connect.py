#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""MQTT Connection Helper Function


"""
from datetime import datetime

import paho.mqtt.client as mqtt
import socket
import time

from pytemi.robot import now


def _on_connect(client, userdata, flags, rc):
    """Connect to MQTT broker and subscribe to topics"""
    print(
        "[STATUS] Connected to: {} (rc:{})".format(
            client._client_id.decode("ascii"), str(rc)
        )
    )

    # subscribing in on_connect() means that if we lose the connection and
    # reconnect, then subscriptions will be renewed
    client.subscribe("temi/#")


def _on_disconnect(client, userdata, rc):
    """Disconnect from MQTT broker"""
    print("DISCONNECTED```````````````````````````````````````")
    print(
        "[STATUS] Disconnected from: {} (rc:{})".format(
            client._client_id.decode("ascii"), str(rc)
        )
    )
    client.loop_stop()


def _on_message(client, userdata, msg):
    """Print out any topics that have no callbacks"""
    print("[{}][SUB] {} {}".format(now(), msg.topic, str(msg.payload)))


def connect(host, port, username=None, password=None):
    """Connect to MQTT broker"""
    client_id = socket.gethostname() + "-" + datetime.now().strftime("%Y%m%d%H%M%S")

    # create a new MQTT client instance
    client = mqtt.Client(client_id=client_id)

    # attach general callbacks
    client.on_connect = _on_connect
    client.on_disconnect = _on_disconnect

    # set username and password
    if username and password:
        client.username_pw_set(username=username, password=password)

    # connect to MQTT broker
    client.connect(host=host, port=port, keepalive=60, bind_address="")

    # start listening to topics
    client.loop_start()

    # wait for client to connect
    # TODO check using on_connect()
    time.sleep(1)

    return client


if __name__ == "__main__":
    # connect to the MQTT server
    mqtt_client = connect("broker.mqttdashboard.com", 1883)
