
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
