# 🤖 ROS Multi-Language Project

This project demonstrates a hybrid **ROS1** system where some nodes are written in **C++** for performance and others in **Python** for rapid development and scripting. It shows how to integrate both languages in a single ROS workspace.

## 📌 Overview

In robotics applications, it's common to split responsibilities across different languages:

- 🧠 **C++ nodes** handle time-critical tasks like sensor processing or motor control.
- 🐍 **Python nodes** handle higher-level logic, prototyping, UI, or scripting.

This repository sets up a basic example with communication between C++ and Python nodes using standard ROS topics.

---

## 🗂 Project Structure

\`\`\`
ros_workspace/
├── src/
│   ├── cpp_node_pkg/
│   │   ├── src/my_cpp_node.cpp
│   │   ├── include/cpp_node_pkg/
│   │   └── CMakeLists.txt
│   ├── python_node_pkg/
│   │   ├── scripts/my_python_node.py
│   │   └── setup.py
├── CMakeLists.txt
└── package.xml
\`\`\`

---

## ⚙️ Dependencies

Make sure you have:

- ROS 1 (e.g., **Noetic** or **Melodic**) installed
- \`catkin\` tools (\`catkin_make\`, \`rosrun\`, \`rospack\`, etc.)
- Python 3 support in ROS (for Noetic)

---

## 🛠️ Building the Workspace

\`\`\`bash
# Clone and enter workspace
git clone https://github.com/yourusername/ros-multi-lang-project.git
cd ros-multi-lang-project

# Source your ROS installation
source /opt/ros/noetic/setup.bash

# Build the workspace
catkin_make

# Source the workspace
source devel/setup.bash
\`\`\`

---

## 🚀 Running the Nodes

### 1. Launch ROS Master

\`\`\`bash
roscore
\`\`\`

### 2. Run C++ Node

\`\`\`bash
rosrun cpp_node_pkg my_cpp_node
\`\`\`

This node will publish sensor-like data to \`/sensor_data\`.

### 3. Run Python Node

Open a new terminal:

\`\`\`bash
source devel/setup.bash
rosrun python_node_pkg my_python_node.py
\`\`\`

This node subscribes to \`/sensor_data\`, processes it, and publishes a response on \`/control_command\`.

---

## 🔁 Topic Communication Example

| Topic              | Type               | Publisher         | Subscriber         |
|--------------------|--------------------|-------------------|--------------------|
| \`/sensor_data\`     | \`std_msgs/Int32\`   | \`cpp_node_pkg\`     | \`python_node_pkg\`  |
| \`/control_command\` | \`std_msgs/String\` | \`python_node_pkg\`  | \`cpp_node_pkg\`      |

You can observe messages with:

\`\`\`bash
rostopic echo /sensor_data
rostopic echo /control_command
\`\`\`

---

## ✅ Testing

To run any unit or integration tests (if added):

\`\`\`bash
catkin_make run_tests
\`\`\`

---


