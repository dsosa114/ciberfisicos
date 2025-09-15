# Final Project Helper Packages

This repository contains essential ROS 2 packages to assist with the final project activity on autonomous mobile robot navigation in a simulated manufacturing cell. The packages provide the core communication interfaces and nodes that simulate the manufacturing environment. **This repository also includes packages created during the course of "Cyber-physical Systems I".**

-----

### 📦 custom\_interfaces

This package defines the custom message interfaces used for communication between the different components in the manufacturing cell simulation. It is crucial for the robot's logic to subscribe and publish to these custom message types.

  * **`Request.msg`**: Sent from a work table to the conveyor to request raw material.
  * **`FinishedJob.msg`**: Sent from a work table to the robot to signal that a product is ready for pickup.
  * **`Supply.msg`**: Sent from the conveyor to the robot to announce that requested material is ready for pickup.
  * **`Delivery.msg`**: Sent from the robot to a work table or conveyor to confirm a delivery has been made.

-----

### 📦 final\_project\_helpers

This package contains the core ROS 2 nodes that simulate the behavior of the manufacturing cell's static components. These nodes are provided as a foundation for your project, allowing you to focus on the robot's navigation and logic.

  * **`table_node.py`**: Simulates the two work tables. They request raw materials and signal when finished products are ready.
  * **`conveyor_node.py`**: Simulates the conveyor belt. It handles requests for raw materials from the tables and announces when supplies are ready for pickup.
  * **`battery_model_node.py`**: Simulates the robot's battery level.

-----

## 🚀 Getting Started

### Prerequisites

To use these packages, you must have **ROS 2 Humble** or **Jazzy** installed on your system.

### Compilation

Follow these steps to clone and compile the packages in your ROS 2 workspace:

1.  Navigate to your ROS 2 workspace's source directory (e.g., `~/ros2_ws/src`).
2.  Clone this repository:
    ```bash
    git clone https://github.com/dsosa114/ciberfisicos.git
    ```
3.  Navigate back to the root of your workspace:
    ```bash
    cd ~/ros2_ws/
    ```
4.  Build the packages using `colcon`:
    ```bash
    colcon build --packages-select custom_interfaces final_project_helpers
    ```
    This command will build only the necessary packages, ensuring a quick and clean compilation. If the build is successful, you are ready to use the helper nodes.

-----

## ⚙️ Usage

The `final_project_helpers` package includes a launch file that simplifies running the simulation nodes.

### Running the Nodes

To launch all the helper nodes (two table nodes, the conveyor node, and the battery model node), use the following command from your workspace's root directory:

```bash
ros2 launch final_project_helpers node_launcher.launch.py
```

This command will start the entire static environment simulation, allowing you to connect your robot's navigation node and begin working on the autonomous logic.

### Node Functionality and Communication

Your robot's navigation node must interact with these helper nodes by subscribing to their published topics and publishing to the topics they subscribe to. Use the custom message interfaces defined in the `custom_interfaces` package for all communication.

  * **Subscribe:** `/battery_state`, `/request`, `/supply`, `/finished_job`
  * **Publish:** `/is_docked`, `/delivery`, `/task_status`

This setup provides a robust and realistic environment for you to test and validate your robot's navigation and decision-making algorithms.