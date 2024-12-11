# Experimental Assignment 1

 The project is designed to [briefly explain the project's main goal or functionality].

## Collaboration
Ali PourYaghoub
Seyed Alireza Mortazavi
Pezhman Rezaei
Mohsen Kashefi

## Features
- Detects markers using their unique IDs.
- Tracks marker locations and aligns the camera view with them.
- Enhances the camera feed by visually marking detected markers.
- Rotates the robot (or camera link) to process multiple markers efficiently.
- Completes rotation and stops once all markers are detected and processed.

---

#### Required
- **Python 3.x**
- **ROS Noetic**


## Installation

Follow these steps to set up the project:

1. Clone this repository:
   ```bash
   git clone https://github.com/MohsenKashefi/experimental_assignment1.git
   ```

2. Navigate to the project directory:
   ```bash
   cd experimental_assignment1
   ```
run the catkin_make required command
   

#### Installation

Install the required Python libraries:
```bash
pip3 install numpy scipy imutils opencv-python
```

### Additional Resources

1. Clone the ArUco ROS repository for marker detection:
```bash
git clone https://github.com/CarmineD8/aruco_ros
```


## Execution

1. **Run the Whole Chassis Rotation Node:**
   ```bash
   roslaunch robot_urdf assignment1.launch
   ```

2. **Run the Camera Link Rotation Node:**
   ```bash
   roslaunch robot_urdf assignment1_2.launch
   ```
   
3. Verify the following topics are active:

   - `/robot/camera1/image_raw/compressed`
   - `/robot/camera1/camera_info`
   - `/marker/id_number`
   - `/marker/center_loc`

4. Observe the robot rotating or adjusting the camera view to detect and process all markers sequentially.

## Topics

### Published Topics

#### `/processed_image/compressed`
#### `/cmd_vel`


### Subscribed Topics

#### `/robot/camera1/image_raw/compressed`
#### `/robot/camera1/camera_info`
#### `/marker/id_number`
#### `/marker/center_loc`







