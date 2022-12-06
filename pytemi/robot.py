#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""temi Robot Class

"""
import json
import re
import time

from datetime import datetime


def now():
    """Return time in string format"""
    return datetime.now().strftime("%H:%M:%S")


def _on_status(client, userdata, msg):
    """Periodically updates the locations in map"""
    d = json.loads(msg.payload)
    userdata["locations"] = d["waypoint_list"]


def _on_battery(client, userdata, msg):
    print("[{}] [SUB] [BATTERY] {}".format(now(), str(msg.payload)))
    # d = "BatteryData(level=65, isCharging=false)"
    d = json.loads(msg.payload)["batteryData"]

    split_string = re.split('[= , ( )]', d)
    userdata["battery"]["percentage"] = float(split_string[2]) / 100
    userdata["battery"]["is_charging"] = split_string[-2]


def _on_goto(client, userdata, msg):
    d = json.loads(msg.payload)
    userdata["goto"]["location"] = d["location"]
    userdata["goto"]["status"] = d["status"]


def _on_user(client, userdata, msg):
    print("[{}] [SUB] [USER] {}".format(now(), str(msg.payload)))
    userdata["user"] = json.loads(msg.payload)


def _on_currentPosition(client, userdata, msg):
    print("[{}] [SUB] [CURRENT POSITION] {}".format(now(), str(msg.payload)))
    split_response = re.split('[= , )]', str(msg.payload))
    userdata["currentPosition"] = {"x": float(split_response[1]),
                                   "y": float(split_response[4]),
                                   "yaw": float(split_response[7]),
                                   "tiltAngle": float(split_response[10])}


def _on_durationToDestination(client, userdata, msg):
    print("[{}] [SUB] [DURATION TO DESTINATION] {}".format(now(), str(msg.payload)))
    userdata["durationToDestination"] = json.loads(msg.payload)

def _on_receiveTestConnection(client, userdata, msg):
    print("[{}] [SUB] [TEST RECEIVE MESSAGE] {}".format(now(), str(msg.payload)))


class Robot:
    """Robot Class"""

    def __init__(self, mqtt_client, temi_serial, silent=True):
        """Constructor"""
        self.client = mqtt_client
        self.id = temi_serial
        self.silent = silent

        # set user data
        # initialized default values for temi robot for location and current position
        self.state = {"locations": ["home base"], "battery": {},
                      "goto": {"location": "home base", "status": "complete"}, "user": {},
                      "currentPosition": {"x": 0.0, "y": 0.0, "yaw": 0.0, "tiltAngle": 50},
                      "durationToDestination": {'duration': 0.0}}
        self.client.user_data_set(self.state)

        # attach subscription callbacks
        self.client.message_callback_add(
            "temi/{}/status/info".format(temi_serial), _on_status
        )
        self.client.message_callback_add(
            "temi/{}/status/utils/battery".format(temi_serial), _on_battery
        )
        self.client.message_callback_add(
            "temi/{}/status/utils/currentPosition".format(temi_serial), _on_currentPosition
        )
        self.client.message_callback_add(
            "temi/{}/status/utils/durationToDestination".format(temi_serial), _on_durationToDestination
        )
        self.client.message_callback_add(
            "temi/{}/event/waypoint/goto".format(temi_serial), _on_goto
        )
        self.client.message_callback_add(
            "temi/{}/event/user/detection".format(temi_serial), _on_user
        )
        self.client.message_callback_add(
            "temi/{}/event/test/testConnection".format(temi_serial), _on_receiveTestConnection
        )

        # call method to initialize battery information
        self.get_battery_data()
        time.sleep(1)
        print('GOT DATA', self.battery)
        self.get_current_position()
        time.sleep(1)

    def checkIfDockingCompleted(self):
        return self.state == "complete" and self.currentLocation == "home base"

    def navigationCompleted(self):
        return self.status == "complete"

    def rotate(self, angle):
        """Rotate"""
        if not self.silent:
            print("[CMD] Rotate: {} [deg]".format(angle))

        if angle != 0:
            topic = "temi/" + self.id + "/command/move/turn_by"
            payload = json.dumps({"angle": angle})

            self.client.publish(topic, payload, qos=0)

    def joystick(self, x, y):
        """Joystick"""
        if not self.silent:
            print("[CMD] Translate: {} {} [unitless]".format(x, y))

        topic = "temi/" + self.id + "/command/move/joystick"
        payload = json.dumps({"x": x, "y": y})

        self.client.publish(topic, payload, qos=0)

    def tilt(self, angle):
        """Tilt head (absolute angle)"""
        if not self.silent:
            print("[CMD] Tilt: {} [deg]".format(angle))

        topic = "temi/" + self.id + "/command/move/tilt"
        payload = json.dumps({"angle": angle})

        self.client.publish(topic, payload, qos=0)

    def tilt_by(self, angle):
        """Tilt head (relative angle)"""
        if not self.silent:
            print("[CMD] Tilt By: {} [deg]".format(angle))

        topic = "temi/" + self.id + "/command/move/tilt_by"
        payload = json.dumps({"angle": angle})

        self.client.publish(topic, payload, qos=0)

    def stop(self):
        """Stop"""
        if not self.silent:
            print("[CMD] Stop")

        topic = "temi/" + self.id + "/command/move/stop"

        self.client.publish(topic, "{}", qos=1)

    def follow(self):
        """Follow"""
        if not self.silent:
            print("[CMD] Follow")

        topic = "temi/" + self.id + "/command/follow/unconstrained"

        self.client.publish(topic, "{}", qos=1)

    def goToLocation(self, location_name):
        self.state["goto"]["location"] = location_name
        self.state["goto"]["status"] = "start"

        """Go to a saved location"""
        if not self.silent:
            print("[CMD] Go-To: {}".format(location_name))

        topic = "temi/" + self.id + "/command/waypoint/goToLocation"
        payload = json.dumps({"location": location_name})

        self.client.publish(topic, payload, qos=1)

    def goToPosition(self, x, y, yaw, tiltAngle=22):
        self.state["goto"]["location"] = "COORDINATES"
        self.state["goto"]["status"] = "start"

        """Go to a position"""
        if not self.silent:
            print("[CMD] Go-To Position:({}, {}), Angle = {} ".format(x, y, yaw))

        topic = "temi/" + self.id + "/command/waypoint/goToPosition"
        payload = json.dumps({"x": x, "y": y, "yaw": yaw, "tiltAngle": tiltAngle})

        self.client.publish(topic, payload, qos=1)

    def tts(self, text):
        # print(self.currentPosition)
        """Text-to-speech"""
        if not self.silent:
            print("[CMD] TTS: {}".format(text))

        topic = "temi/" + self.id + "/command/tts"
        payload = json.dumps({"utterance": text})

        self.client.publish(topic, payload, qos=1)

    def video(self, url):
        """Play video"""
        if not self.silent:
            print("[CMD] Play Video: {}".format(url))

        topic = "temi/" + self.id + "/command/media/video"
        payload = json.dumps({"url": url})

        self.client.publish(topic, payload, qos=1)

    def webview(self, url):
        """Show webview"""
        if not self.silent:
            print("[CMD] Show Webview: {}".format(url))

        topic = "temi/" + self.id + "/command/media/webview"
        payload = json.dumps({"url": url})

        self.client.publish(topic, payload, qos=1)

    def custom(self, topic, data):
        """Send custom message"""
        if not self.silent:
            print("[CMD] Custom")

        topic = "temi/" + self.id + topic
        payload = json.dumps(data)

        self.client.publish(topic, payload, qos=1)

    def get_battery_data(self):
        """Get Battery Data"""
        if not self.silent:
            print("[CMD] Get Battery Data")

        topic = "temi/" + self.id + "/command/getData/batteryData"

        try:
            self.client.publish(topic, "{}", qos=1)
        except Exception as e:
            print("Exception received when getting battery data! ", e)

    def get_current_position(self):
        """Get Current Position"""
        if not self.silent:
            print("[CMD] Get Current Position")

        topic = "temi/" + self.id + "/command/getData/currentPosition"

        self.client.publish(topic, "{}", qos=1)

    @property
    def locations(self):
        """Return a list of locations"""
        if "locations" in self.state:
            return self.state["locations"]
        else:
            return []

    @property
    def status(self):
        if "status" in self.state["goto"]:
            return self.state["goto"]["status"]
        else:
            return None

    @property
    def currentLocation(self):
        if "location" in self.state["goto"]:
            return self.state["goto"]["location"]
        else:
            return None

    @property
    def battery(self):
        return self.state["battery"]

    @property
    def currentPosition(self):
        return self.state["currentPosition"]

    @property
    def durationToDestination(self):
        return self.state["durationToDestination"]

    @property
    def GOTO_START(self):
        return "start"

    @property
    def GOTO_ABORT(self):
        return "abort"

    @property
    def GOTO_GOING(self):
        return "going"

    @property
    def GOTO_COMPLETE(self):
        return "complete"

    @property
    def GOTO_CALCULATING(self):
        return "calculating"

    @property
    def GOTO_OBSTACLE(self):
        return "obstacle detected"
