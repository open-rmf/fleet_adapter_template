# fleet_adapter_template

> Note: If you are using Open-RMF binaries from ROS 2 Humble or an older distribution, switch to the [humble](https://github.com/open-rmf/fleet_adapter_template/tree/humble) branch.

The objective of this package is to serve as a reference or template for writing a python based `full_control` RMF fleet adapter.

> Note: The implementation in this package is not the only way to write a `full_control` fleet adapter. It is only one such example that may be helpful for users to quickly integrate their fleets with RMF.

## Step 1: Fill up missing code
Simply fill up certain blocks of code which make API calls to your mobile robotic fleet.
These blocks are highlighted as seen below and are found in `RobotClientAPI.py` and `fleet_adapter.py` respectively.
```
# IMPLEMENT YOUR CODE HERE #
```

The bulk of the work is in populating the `RobotClientAPI.py` file which defines a wrapper for communicating with the fleet of interest.
For example, if your fleet offers a `REST API` with a `GET` method to obtain the position of the robot, then the `RobotAPI::position()` function may be implemented as below

```python
def position(self):
    url = self.prefix + "/data/position" # example endpoint
    try:
        response = requests.get(url)
        response.raise_for_status()
        data = response.json()
        x = data["x"]
        y = data["y"]
        angle = data["angle"]
        return [x, y, angle]
    except HTTPError as http_err:
        print(f"HTTP error: {http_err}")
    except Exception as err:
        print(f"Other error: {err}")
    return None

```

Alternatively, if your robotic fleet offers a websocket port for communication or allows for messages to be exchanged over ROS1/2, then these functions can be implemented using those protocols respectively.

## Step 2: Update config.yaml
The `config.yaml` file contains important parameters for setting up the fleet adapter. There are three broad sections to this file:

1. **rmf_fleet** : containing parameters that describe the robots in this fleet
2. **fleet_manager** : containing configurations to connect to the robot's API in order to retrieve robot status and send commands from RMF
3. **reference_coordinates**: containing two sets of [x,y] coordinates that correspond to the same locations but recorded in RMF (`traffic_editor`) and robot specific coordinates frames respectively. These are required to estimate coordinate transformations from one frame to another. A minimum of 4 matching waypoints is recommended.

> Note: This fleet adapter uses the `nudged` python library to compute transformations from RMF to Robot frame and vice versa. If the user is aware of the `scale`, `rotation` and `translation` values for each transform, they may modify the code in `fleet_adapter.py` to directly create the `nudged` transform objects from these values.

## Step 3: Run the fleet adapter:

Run the command below while passing the paths to the configuration file and navigation graph that this fleet operates on.

The websocket server URI should also be passed as a parameter in this command inorder to publish task statuses to the rest of the RMF entities.

```bash
#minimal required parameters
ros2 run fleet_adapter_template fleet_adapter -c CONFIG_FILE -n NAV_GRAPH

#Usage with the websocket uri
ros2 run fleet_adapter_template fleet_adapter -c CONFIG_FILE -n NAV_GRAPH -s SERVER_URI

#e.g.
ros2 run fleet_adapter_template fleet_adapter -c CONFIG_FILE -n NAV_GRAPH -s ws://localhost:7878
```
