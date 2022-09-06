--
author:
- Olmer Garcia-Bedoya
title: ROS Introduction
---

Introduction
============

Robot Operating System (ROS) is an open source software framework for
robot development, including middleware, drivers, libraries, tools, and
commonly used algorithms for robotics. ROS was originally designed for
use in the education research communities, but robotics companies use
ROS as a framework for robotics product development. Because ROS wasn't
designed with production robot systems in mind, companies must put in
time and effort to productize their internal forks of various ROS
components.

ROS 2 is the second generation of ROS, redesigned from the ground up.
ROS 2 seeks to address prior ROS shortcomings, reducing the complexity
of converting ROS 2 based prototypes into products. An original "Why ROS
2?" article from the ROS 2 design documents website, gives background
and rationale for the ROS 2 re-design decision.

ROS 2 provides new capabilities that ROS originally did not. Beginning with a layered architecture that separates the ROS client layer (RCL)
from the ROS middleware layer (RMW). The RCL provides the developer
interface and the RMW layer enables compatibility with different
interchangeable low-level communication protocols. The RMW is built on
top of the Data Distribution Service (DDS), a certifiable real-time
publish/subscribe protocol designed for safety critical systems. DDS makes ROS 2 a more robust and interoperable framework. This architecture separates the lowest level protocol (DDS) details, from the application developer layer. Layered abstractions allow developers to focus on their unique application, algorithm, or driver instead of the underlying
details.

![ROS 2 layered architecture. from
[amazon](https://aws.amazon.com/es/blogs/robotics/ros-2-foxy-fitzroy-robot-development/)amazon](image-2.png)

ROS 2 also adds new advanced features, like Quality of Service (QoS) settings to tune communication between processes, and lifecycle nodes for managing deterministic system startup and shut down. In addition, ROS2 relies on more up to date C++ 14 and Python 3 language standards and libraries. ROS 2 Foxy is the latest LTS version and it is defined by aws amazon like ["the most secure and reliable ROS distribution to date for production robotics application development"](https://aws.amazon.com/es/blogs/robotics/ros-2-foxy-fitzroy-robot-development/)"the
most secure and reliable ROS distribution to date for production
robotics application development".

In this section is presented the ROS concepts divided in three concept that implement ROS: Communications between program or nodes(plumbing), a cross platform set of libraries to implements program for robots and a
tooling set to debug and manipulate the information of a robot.

Communication
-------------

-   **nodes.** One node is basically a program or executable **.** Each     node in ROS should be responsible for a single, module purpose (e.g.     one node for controlling wheel motors, one node for controlling a     laser range-finder, etc). Each node can send and receive data to
    other nodes via topics, services, actions, or parameters. A full
    robotic system is comprised of many nodes working in concert. In ROS
    2, a single executable (C++ program, Python program, etc.) can
    contain one or more nodes.

-   **Topics.** Topics are a vital element of the ROS graph that act as
    a bus for nodes to exchange messages. A node may publish data to any
    number of topics and simultaneously have subscriptions to any number
    of topics. Topics are one of the important ways that data moves
    between nodes, and therefore between different parts of the system.

-   **Services.** Services are another method of communication for nodes
    on the ROS graph. Services are based on a call-and-response model,
    versus topics' publisher-subscriber model. While topics allow nodes
    to subscribe to data streams and get continual updates, services
    only provide data when they are specifically called by a client

-   **Action.** Actions are one of the communication types in ROS 2
    intended for long running tasks. They consist of three parts: a
    goal, a result, and feedback. Actions are built on topics and
    services. Their functionality is similar to services, except actions
    are preemptable (you can cancel them while executing). They also
    provide steady feedback, as opposed to services which return a
    single response.

-   **Parameters.** A parameter is a configuration value of a node. You
    can think of parameters as node settings. A node can store
    parameters as integers, floats, booleans, strings and lists. In ROS
    2, each node maintains its own parameters. All parameters are
    dynamically reconfigurable, and built off of ROS 2 services.

ROS 2 offers a rich variety of Quality of Service (QoS) policies that
allow you to tune communication between nodes. With the right set of
Quality of Service policies, ROS 2 can be as reliable as TCP or as
best-effort as UDP, with many, many possible states in between. In this
point I invite you to read [About Quality of service](https://index.ros.org/doc/ros2/Concepts/About-Quality-of-Service-Settings/)
where are explained the QoS policies, QoS profiles, QoS compatibilities and QoS events available inside ROS2.

Development
-----------

-   **workspace.** "Workspace" is a ROS term for the location on your
    system where you're developing with ROS 2. The core ROS 2 workspace
    is called the underlay. Subsequent local workspaces are called
    overlays. When developing with ROS 2, you will typically have
    several workspaces active concurrently.

-   **package**. A package can be considered a container for your ROS 2
    code. If you want to be able to install your code or share it with
    others, then you'll need it organized in a package. With packages,
    you can release your ROS 2 work and allow others to build and use it
    easily.

-   **launch**. Launch files allow you to start up and configure a
    number of executables containing ROS 2 nodes simultaneously. Running
    a single launch file with the *ros2 launch* command will start up
    your entire system - all nodes and their configurations - at once.

-   **[urdf.](http://wiki.ros.org/urdf%20)** . The Unified Robot
    Description Format (URDF) is an XML specification to describe a
    robot. The specification covers: Kinematic and dynamic description
    of the robot, Visual representation of the robot and Collision model
    of the robot.

Tools
-----

-   **CLI tools**. A command-line interface (CLI) processes commands to
    a computer program in the form of lines of text. All CLI command
    start with ros2 following by what do you want to affect (run,
    launch, topics,service, parameters) and how do you want (list, echo,
    call,\...). Always you can give the term -h to obtain help about the
    command.

-   **rqt**. rqt is a GUI tool for ROS 2. Everything done in rqt can be
    done on the command line, but it provides an easier, more
    user-friendly way to manipulate ROS 2 elements.

    -   **[rqt_bash](https://index.ros.org/p/rqt_bash/)**.
        rqt_bash provides a GUI plugin for displaying and filtering
        ROS messages. ROS 2's logger levels are ordered by severity:
        Fatal, Error, Warn, Info and Debug.

-   **rosbag**. It is a command line tool for recording data published
    on topics in your system. It accumulates the data passed on any
    number of topics and saves it in a database. You can then replay the
    data to reproduce the results of your tests and experiments.
    Recording topics is also a great way to share your work and allow
    others to recreate it.Ferramentas

-   **rviz**. 3D visualization tool for ROS. The robot will be
    visualized here to determine in one interface how the perception is
    working in the robot and in some case give the mission to the robot.

Ros Foxy Docker
=====================

TODO: explain



Turtlesim 
=========

Turtlesim is a lightweight simulator for learning ROS 2. It illustrates what ROS 2 does at the most basic level, to give you an idea of what you will do with a real robot or robot simulation later on. This simulator has some nodes , which you can review with the command
`ros2 pkg executables turtlesim` . here we are goin two use two:

-   turtlesim turtlesim_node. This is the simulator , which open the
    turtle in xy enviroment

-   turtlesim turtle_teleop_key. This let to manipulate the turtlesim
    with the keyboard sending commands thrown topics.

For run a node in ros2, you should open a bash and write *ros2 run namePackage nameNode.* For our exercise we are going to open two bash
and execute one node in each bash, adittionally we are going open in
a third bash for execute *rqt*

-   **bash 1**. *ros2 run turtlesim turtlesim_node*

-   **bash 2.** *ros2 run turtlesim turtle_teleop_key*

-   **bash 3.** *rqt .* Inside this app go to
    plugins-\>Introspection-Node graph

![image](turtlesim1){width="1\\columnwidth"}

After you open this nodes or program you can manipulate the turtle while
have active the bash2 the turtle running in the bash1, that is
performed because the teleop node publish a command called
/turtle1/cmd_vel through a topic with type geometry_msgs/msg/twist,
compose by a vector of the angular speed and the linear speed of the
turtle. To understand this, I invite you, inside the
*rqt-\>plugins-\>Topics Monitor* to review the structure of the message,
and test what happen when you move the robot through the arrows (in
bash2), which value are send to the turtle slecting this topic in
*rqt.* Other nice exercise is try to manipulate the turtle from
*rqt-\>plugins-\>Topics publisher.* Here you can make that the turtle
generate a circle giving a linear speed in $x$ and an angular speed in
$z$.

Some information of the simulator can be manipulated through services,
for example, imagine that you which to clear the floor of the simulation
or add a new turtle to the simulation. This action can be performed by
rqt through *Plugins \> Services \> Service Caller.* Experiment with
services like */clear, /reset, /turtle1/sim,/turtle1/teleport_absolute
and /turtle1/teleport_relative.* First modify the parameters and then
put *call.* Finally I invite you to revise the tutorial [turtlesim and
rqt](https://index.ros.org/doc/ros2/Tutorials/Turtlesim/Introducing-Turtlesim/#try-the-spawn-service)
to create more that one turtle using the service */spawn , note that new
topics and services* are created for manipulated the turtle2. Try to
explore the concept *remapping* which let to make mirrors between
topics.

If you prefer CLI tools, you can make all that we make with *rqt* trown
commands, you can review *ros2 topic -h , ros2 service -h , ros2 node -h
or ros2 run -h or review the tutorials
[rosnode](https://index.ros.org/doc/ros2/Tutorials/Understanding-ROS2-Nodes/#ros2nodes),
[rostopic](https://index.ros.org/doc/ros2/Tutorials/Topics/Understanding-ROS2-Topics/#ros2topics)
[rosparameters](https://index.ros.org/doc/ros2/Tutorials/Parameters/Understanding-ROS2-Parameters/#ros2params)
and
[rosservice](https://index.ros.org/doc/ros2/Tutorials/Services/Understanding-ROS2-Services/#ros2services).
Some examples:*

-   ros2 node list (List the nodes running in the enviroment)

-   *ros2 node info /turtlesim (review the information of which
    communication forms has a node)*

-   ros2 topic echo /turtle1/cmd/vel (suscribe to specific topic to
    print in bash)

-   *ros2 topic list -t (list the topics availables with the message
    type)*

Node that the turtle also can communicate through an action. This
communicatino work in the architecture of client-server , where the
turtle is the server and the teleop is the client. You can make petition
with letter keys G\|B\|V\|C\|D\|E\|R\|T to make to move to some
orientation the turtle(*goal*). The F key will cancel a goal
mid-execution, demonstrating the preemptable feature of actions. Other
form to review this concept is change the goal during the execution of
other task. I invite to review bash1 and bash2 to check message.
More informataion review the turorial rosactions. If you want to send an
action to the turtle you can test:

*ros2 action send_goal /turtle1/rotate_absolute
turtlesim/action/RotateAbsolute {theta: -1.57} \--feedback*

There exist other way to configure a node which is the parameters, here
information like backgrouund color can be changed, his information can
be changed in three way: through a [yaml
file](https://en.wikipedia.org/wiki/YAML), the cli command *ros2 param
-h* or in some cases though the rqt in
*plugins-\>configuration-\>Dynamic reconfigure.* Usually this data is
use to configure the specific ip of a robot or the name of the robot.

All the information printed in bash of each node is managed by a
topic. This ROS 2's logger levels are ordered by severity: Fatal, Error,
Warn, Info and Debug, to know more about it please visit the tutorial
[ROSbash](https://index.ros.org/doc/ros2/Tutorials/Rqt-bash/Using-Rqt-bash/#rqt-bash).
Other insteresting tool is ROSBAG, which let to record all the
communication between nodes, which lets that you can then replay the
data to reproduce the results of your tests and experiments. I recommend
the tutorial
[ROSBAG](https://index.ros.org/doc/ros2/Tutorials/Ros2bag/Recording-And-Playing-Back-Data/#ros2bag)
and if you want to review advanced like quality of services(QoS) view
[ROSBAGQOS](https://index.ros.org/doc/ros2/Tutorials/Ros2bag/Overriding-QoS-Policies-For-Recording-And-Playback/#ros2bag-qos-override).

Manage many bash in a real scenary is not a good approach, to solve
this problem ros has the option to create a launch file which running a
single bash with the *ros2 launch* command will start up your entire
system - all nodes and their configurations - at once. For our tutorial
good start is
[ROSLAUNCH](https://index.ros.org/doc/ros2/Tutorials/Launch-Files/Creating-Launch-Files/#ros2launch).
