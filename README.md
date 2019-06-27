# cuboid_sim
## 説明
- CuboidくんのROSシミュレーション環境(Gazebo)です
- gazebo環境にて、地図の作成、自律移動を試すことができます
## Instration

### Requirements
  - Ubuntu (>16.04)
  - ROS Kinetic
  - Gazebo(>7.0)
### build
  - `$ mkdir -p ~/ros/src && cd ~/ros/src/`
  - `$ git clone git@github.com:sbgisen/cuboid_sim.git`
  - `$ cd ~/ros/`
  - `$ rosdep init`
  - `$ rosdep install -y --from-paths src --ignore-src -r --rosdistro kinetic`
  - `$ catkin init`
  - `$ catkin build`
### 困った時は
  - rosdepがないって怒られる
     - `$ sudo apt install python-rosdep` でインストール
  - ROS Kineticがはいらない
     - [ROS wiki](http://wiki.ros.org/kinetic/Installation/Ubuntu)を参照
  - git cloneができない
     - `$ sudo apt install git`

## How to use

### Gazeboの起動
- (@terminal1)`$ roslaunch cuboid_gazebo robot_office.launch`
- (@terminal2)`$ rosservice call /gazebo/unpause_physics`

### Keyboardによる操作
- (@terminal2)`$ roslaunch cuboid_teleop teleop_key.launch`
- ->terminalに表示されるように'i','j','l',','などで前後、旋回の操作可能

### Mapの作成
- `$roslaunch cuboid_nav exploration_demo.launch`
- -> 

### 自律移動
- `$roslaunch cuboid_nav navigation.launch`
- -> 
