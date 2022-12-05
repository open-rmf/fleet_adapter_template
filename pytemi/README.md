# MQTT Topics
The following is a summary of MQTT topics used by this package.

## Publish
```
temi/{id}/command/move/turn_by
temi/{id}/command/move/joystick
temi/{id}/command/move/tilt
temi/{id}/command/move/tilt_by
temi/{id}/command/move/stop
temi/{id}/command/follow/unconstrained
temi/{id}/command/waypoint/goto
temi/{id}/command/tts
temi/{id}/command/media/video
temi/{id}/command/media/webview
```

## Subscribe
```
temi/{id}/status/info
temi/{id}/status/utils/battery
temi/{id}/event/waypoint/goto
temi/{id}/event/user/detection
```
