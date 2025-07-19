
# 🤖 ROS Multi-Language Project

This project demonstrates a hybrid **ROS1** system where some nodes are written in **C++** for performance and others in **Python** for rapid development and scripting. It shows how to integrate both languages in a single ROS workspace.

---

## 📌 Overview

In robotics applications, it's common to split responsibilities across different languages:

- 🧠 **C++ nodes** handle time-critical tasks like sensor processing or motor control.
- 🐍 **Python nodes** handle higher-level logic, prototyping, UI, or scripting.

This repository sets up a basic example with communication between C++ and Python nodes using standard ROS topics.

---

## 🧩 Project Layout

```
ros_workspace/
├── src/
│   ├── cpp_node_pkg/
│   │   ├── include/cpp_node_pkg/
│   │   ├── src/my_cpp_node.cpp
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── python_node_pkg/
│   │   ├── scripts/my_python_node.py
│   │   ├── setup.py
│   │   └── package.xml
│   └── multi_launch/
│       └── launch/
│           └── system.launch
├── CMakeLists.txt
└── package.xml
```

---

## ⚙️ Dependencies

Make sure you have:

- ROS 1 (e.g., **Noetic** or **Melodic**) installed
- `catkin` tools (`catkin_make`, `rosrun`, `rospack`, etc.)
- Python 3 support in ROS (for Noetic)

---

## 🛠️ Building the Workspace

```bash
# Clone the project
git clone https://github.com/yourusername/ros-multi-lang-project.git
cd ros-multi-lang-project

# Source ROS setup
source /opt/ros/noetic/setup.bash

# Build
catkin_make

# Source your workspace
source devel/setup.bash
```

---

## 🚀 Running the System

### Option 1: Run Nodes Manually

```bash
roscore
```

Then, in new terminals:

```bash
source devel/setup.bash
rosrun cpp_node_pkg my_cpp_node
```

```bash
source devel/setup.bash
rosrun python_node_pkg my_python_node.py
```

### Option 2: Launch Everything

```bash
roslaunch multi_launch system.launch
```

This launch file starts both C++ and Python nodes automatically.

---

## 🔁 Topics Used

| Topic              | Type               | Publisher         | Subscriber         |
|--------------------|--------------------|-------------------|--------------------|
| `/sensor_data`     | `std_msgs/Int32`   | `cpp_node_pkg`     | `python_node_pkg`  |
| `/control_command` | `std_msgs/String` | `python_node_pkg`  | `cpp_node_pkg`      |

You can observe messages with:

```bash
rostopic echo /sensor_data
rostopic echo /control_command
```

---

## 🧪 Testing

If you have tests defined in the packages:

```bash
catkin_make run_tests
```

---

## 📄 License

This project is licensed under the **MIT License**.  
Feel free to reuse, modify, and build upon it.

---

© 2025 ROS Multi-Language Contributors


---

## 🧠 Advanced Details

### 🔄 Message Flow

This example simulates a simple flow:
1. The **C++ node** publishes a counter on `/sensor_data` every second.
2. The **Python node** listens to `/sensor_data`, logs the data, and responds with a message on `/control_command`.
3. The **C++ node** can optionally subscribe to `/control_command` to act upon it.

This simulates real robotic systems where sensors are polled, then logic is applied, and then commands are sent back.

### 🧱 Custom Message Types (Optional)

You can define your own `.msg` files if needed.

Example:
```bash
roscd cpp_node_pkg/msg
echo -e "int32 id
float32 temperature" > Sensor.msg
```

Update `package.xml` and `CMakeLists.txt` accordingly.

### 🧪 Unit Test Example (Python)

You can test your Python node with `rostest`:

**test/test_sensor_processing.py**
```python
import unittest
import rospy
from std_msgs.msg import Int32

class TestSensorProcessing(unittest.TestCase):
    def test_basic_logic(self):
        # Example test logic
        self.assertTrue(1 + 1 == 2)
```

Then add this test in `CMakeLists.txt` or run via:
```bash
rostest python_node_pkg test_sensor_processing.test
```

---

## 🧰 Useful ROS Commands

| Command                            | Description                              |
|------------------------------------|------------------------------------------|
| `rosnode list`                     | List active nodes                        |
| `rostopic list`                    | List all topics                          |
| `rostopic echo /topic_name`        | Print topic messages                     |
| `rosrun rqt_graph rqt_graph`       | Visualize ROS node connections           |
| `roslaunch pkg_name file.launch`   | Launch multiple nodes                    |
| `rosmsg show std_msgs/String`      | Show message format                      |
| `rosservice list`                  | List active services                     |
| `rosparam get /param_name`         | Get parameter from parameter server      |

---

## 🚀 Suggested Enhancements

- Add **parameter server** usage to make nodes configurable without code changes.
- Use **actionlib** if you need goal-feedback-result patterns (e.g., motion planning).
- Integrate with **RViz** or **rqt_plot** for visual debugging.
- Add **logging** and **error handling** to make the system robust.

---

## 🌐 Useful Links

- [ROS Wiki](http://wiki.ros.org)
- [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)
- [catkin tools](https://catkin-tools.readthedocs.io/en/latest/)
- [rqt_graph](http://wiki.ros.org/rqt_graph)

