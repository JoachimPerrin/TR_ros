# TP1 - initialisation ROS2

### CLI

First of all, we open 3 consoles. The first one for the turtlesim Gui: `ros2 run turtlesim turtlesim_node`.  
The second one for the keyboard teleoperation: `ros2 run turtlesim turtle_teleop_key`.  
We can already see that we can control the turtle in the former node thanks to this one. The last terminal is then used for the CLI

1. Controlling the turtle with the CLI

```console
$ros2 topic list
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```
We see that `cmd_vel` is the topic responsible for the command transmission.

```console
$ ros2 topic info /turtle1/cmd_vel
Type: geometry_msgs/msg/Twist
Publisher count: 1
Subscription count: 1
```
The type of messages on this topic is `geometry_msgs/msg/Twist`

```console
$ ros2 interface show geometry_msgs/msg/Twist
# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
	float64 x
	float64 y
	float64 z
Vector3  angular
	float64 x
	float64 y
	float64 z
```
Here we see how `geometry_msgs/msg/Twist` are structured.

```console
$ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 1.0}, angular: {x: 2.0, y: 0.0,z: 0.8}}"
```
Now the turtle1 start spinning (indefinitely as this command continuously publish by default)


**Un topic est un moyen de communication entre deux noeuds à sens unique, asynchrone, et continu. Il peut acceuillir plusieurs publishers et plusieurs listeners. Lorsqu'un publisher dépose un message sur un topic, chaque listener le reçoit au moyen d'un callback.**

2. Enable/Disable the pen

Just like before, we see through `ros2 service list` that the service responsible for the pen is `/turtle1/set_pen`. We can inspect it thanks to `ros2 service info /turtle1/set_pen`, letting us know that the expected type is: `turtlesim/srv/SetPen`. I struggled understanding the expectation for the field off (`ros2 interface show turtlesim/srv/SetPen`), but found out about `ros2 interface proto turtlesim/srv/SetPen`. I was then able to disable the pen using:  
`ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{'off': 1}"`

**Le service est un moyen de communication ponctuel et synchrone. Un noeud peut proposer un service, qui sera accessible par les autres. Lorsqu'un noeud veut y faire appel, il envoie une requète au noeud server, qui effectue la tâche et renvoie une reponse au noeud client.**

3. Setting parameters

Again, we can use about the same commands as before with `ros2 param <command>`. Thanks to `list`, we observe the following parameters:  
```console
/turtlesim:
  background_b
  background_g
  background_r
```

And changing the color is therefor as simple as `ros2 param set /turtlesim background_b 0` for example.

**Les rosparams sont des constantes appartenant à un node, modifiable au runtime. Contrairement aux topics et services, ce ne sont pas des outils de communication, mais plutôt de configuration.**

### Programmation

Starting with the following actions:
```console
cd ros2_ws/src
ros2 pkg create --build-type ament_cmake --license Apache-2.0 turtle_custom --node-name custom_sub
mkdir turtle_custom/launch

```