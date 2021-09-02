Programming in ROS2
===================

based in [ros docs-foxy](https://docs.ros.org/en/foxy/Tutorials.html#beginner-client-libraries)

Create Workspace
----------------

-   Install required

    *sudo apt install python3-colcon-common-extensions*

    *sudo apt-get install python3-rosdep2*

-   Create Workspace

    *mkdir -p \~/utadeo_ws/src*

    *cd \~/utadeo_ws/src*

-   Build

    colcon build

Underlay Vs overLay

*echo source \~/utadeo_ws/install/setup.bash \>\> \~/.bashrc*

Programming in Python
----------------

- Create a package:
	cd ~/utadeo_ws/src
	ros2 pkg create --build-type ament_python py_basic_examples
- Create the python file min_publisher.py in folder ~/utadeo_ws/src/py_basic_examples/py_basic_examples

```
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'menssage', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    # Destroy the node explicitly(optional)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
- Create the python file min_subscriber.py in the folder ~/utadeo_ws/src/py_basic_examples/py_basic_examples
```
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'menssage',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    # Destroy the node explicitly (optional)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

- open the file ~/utadeo_ws/src/py_basic_examples/package.xml and modify
```
    <name>py_basic_examples</name>
    <version>0.0.0</version>
    <description>TODO: Package description</description>
    <maintainer email="robot@todo.todo">robot</maintainer>
    <license>TODO: License declaration</license>
```

- configure the executable in the file ~/utadeo_ws/src/py_basic_examples/setup.py
```
entry_points={
        'console_scripts': [
                'talker = py_basic_examples.min_publisher:main',
                'listener = py_basic_examples.min_subscriber:main',
        ],
},
```
- the file setup.cfg configure for ros2 where is the package(do not change)

- You should have installed the packages (*rclpy* e *std_msgs*), but is better to execute in console `rosdep`.
	cd ~/utadeo_ws/
	rosdep install -i --from-path src/py_basic_examples --rosdistro foxy -y

- compile for generate the link to the  script with ros2
	cd ~/utadeo_ws/
	colcon build --packages-select py_basic_examples


## testing

- **console one** publisher 
	cd ~/utadeo_ws
	. install/setup.bash
	ros2 run py_basic_examples talker
	
- **console two** suscriber 
	cd ~/utadeo_ws
	. install/setup.bash
	ros2 run py_basic_examples listener

- **console three** review the node graph with rqt
    rqt

 
## Connecting to Turtlesim (our robot)

We are going to modify the publisher to move the robot in circle.

Modify th file min_publisher.py to circle_turtle.py :

- add library to twist message
	from geometry_msgs.msg import Twist
- change node name:
	super().__init__('circle_turtle')
- change type of message and name of topy
	self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

- modify the function `timer_callback` to send the required type of message.
```
    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0.5
        self.publisher_.publish(msg)
        self.get_logger().info(f'linear vel: {msg.linear.x}, angular vel {msg.angular.z}' )
        self.i += 1
```
- modify  *package.xml* to add the package `geometry_msgs`
 
	<exec_depend>geometry_msgs<exec_depend>

- add the executable in *setup.py*
```
'console_scripts': [
                'talker = py_basic_examples.min_publisher:main',
                'listener = py_basic_examples.min_subscriber:main',
                'circle_turtle = py_basic_examples.circle_turtle:main',
        ],
```
- execute the script

	cd ~/utadeo_ws
	. install/setup.bash
	ros2 run py_basic_examples circle_turtle

- execute the robot
	ros2 run turtlesim turtlesim_node --ros-args --remap turtle1/cmd_vel:=/cmd_vel


