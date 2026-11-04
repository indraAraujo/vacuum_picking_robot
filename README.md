# Robô Aspirador com Garra e Interação Humano-Robô

Arquitetura embarcada para um robô aspirador com garra capaz de pegar lixo, mapear ambientes e interagir com seres humanos.

## Pacote de Navegação com Nav2

### Nós do Pacote

| Node | Role | Key I/O |
| --- | --- | --- |
| `myrobot_server` | Simulated mobile base publishing telemetry. | Publishes docking beacon signal, relative angle, and `sensor_msgs/BatteryState`; subscribes to `/myRobot/cmd_vel` commands and behaviour mode. |
| `behaviour_server` | Central operational state machine. | Subscribes to battery %, docking strength, and `/myRobot/start_stop`; publishes `/myRobot/behaviour/mode`; logs every transition plus periodic battery summaries. |
| `cleaning_server` | Emits a simple cleaning twist when mode is `CLEANING`. | Watches behaviour mode, publishes `/myRobot/cmd_vel`. |
| `autodocking_server` | Drives toward the charging base when mode is `AUTODOCKING`. | Consumes docking signal topics and publishes `/myRobot/cmd_vel`. |
| `charging_server` | Stub for charge supervision logic. | Listens to battery + behaviour topics; place charge control logic here. |
| `charged_server` | Placeholder for maintenance tasks while docked. | Subscribes to behaviour mode. |
| `nav2_lifecycle_manager` | Configures and activates every node listed in `config/lifecycle_manager.yaml`. | Ensures the lifecycle state is managed independently from operational behaviour. |

Every node uses `QoSProfile(depth=1)` so transitions propagate quickly without
back pressure. All topics/services live under the `/myRobot` namespace for
clarity.

### Máquina de Estados de Comportamentos

`behaviour_server` owns the operational state. It initializes in `CHARGED`, then
uses battery telemetry, “near dock” signal strength, and the Start/Stop service
to decide mode transitions.

```
Start-up -> CHARGED
CHARGED --(Start command)--> CLEANING
CLEANING --(Battery <= 25%)--> AUTODOCKING
AUTODOCKING --(Dock signal strong)--> CHARGING
CHARGING --(Battery >= 99.9% & stop requested)--> CHARGED
CHARGING --(Battery >= 99.9% & no stop)--> CLEANING
CLEANING --(Stop command)--> AUTODOCKING
```

Implementation highlights:

- `BATTERY_LOW` is set to `0.25`. Crossing that threshold triggers automatic
  docking if the robot is cleaning.
- `_set_mode()` centralizes transitions and emits a log of the form
  `Behaviour mode CLEANING -> AUTODOCKING (battery low (0.24))` so you can trace
  *why* modes change.
- `_log_battery_status()` fires every 5 s to log
  `Battery status: 75.0% | mode=CLEANING | near_base=False | stop_requested=False`.
- During `on_configure()` the node logs the initial state snapshot so you know
  the starting mode and telemetry values when the application boots.

Depending on the mode, the other nodes either publish motion commands or remain
idle. Because lifecycle activation never changes, you can add actuators and
sensors to those nodes without worrying about state churn.

### Execução da Navegação


```bash
cd ~/myrobot_bringup_ws
colcon build --packages-select myrobot_interfaces myrobot_bringup
set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u
ros2 launch myrobot_bringup bringup.launch.py
```

`bringup.launch.py` starts all six robot nodes plus Nav2's lifecycle manager.
Watch the console: you should see the lifecycle “configured/activated” logs
followed by the behaviour server’s initial snapshot and periodic battery data.

### Interação com a execução

- **Start cleaning** (from dock):

  ```bash
  cd ~/myrobot_bringup_ws
  set +u
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  set -u
  
  ros2 service call /myRobot/start_stop myrobot_interfaces/srv/StartStop '{command: "start"}'
  ```

- **Request a stop** (forces auto docking when currently cleaning):

  ```bash
  cd ~/myrobot_bringup_ws
  set +u
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  set -u
 
  ros2 service call /myRobot/start_stop myrobot_interfaces/srv/StartStop '{command: "stop"}'
  ```

- **Monitor logs**: `ros2 log echo /rosout` or use `grep` on the node output to
  see transition reasons and battery summaries.

- **Inspect topics**: `ros2 topic echo /myRobot/behaviour/mode` or
  `/myRobot/battery_state` to visualize the state machine working.

Because `myrobot_server` simulates the docking signal and battery level, you can
observe the entire cycle (clean → dock → charge → charged) without extra
hardware.

## Operações do CLI

**Essential commands**

- Check whether the stack is active (optional):

  ```bash
  ros2 service call /lifecycle_manager/is_active std_srvs/srv/Trigger "{}"
  ```

  If `success: true`, the Lifecycle Manager considers the system “active”. (`/lifecycle_manager/is_active` is a `std_srvs/Trigger`; see the Nav2 API docs for details.)

- Shut down the entire stack gracefully:

  ```bash
  # SHUTDOWN = 4 on Jazzy
  ros2 service call /lifecycle_manager/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 4}"
  ```

- Alternative sequences (when needed):

  - Pause / resume without shutting down:

    ```bash
    ros2 service call /lifecycle_manager/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 1}"  # PAUSE
    ros2 service call /lifecycle_manager/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 2}"  # RESUME
    ```

  - Reset (deactivate, clean up, reconfigure/reactivate):

    ```bash
    ros2 service call /lifecycle_manager/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 3}"
    ```

## Referências

- [Base de arquitetura para navegação por Luciano](https://github.com/lucianovk/IA368II/tree/main)