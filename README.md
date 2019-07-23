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
- -> rvizの`2D Nav Goal`を置くことでゴール地点を指定し、地図を作成する
- `$ rosrun map_server map_saver -f __/path/to/map-file__`
- -> __/path/to/map-file__ にMapファイルを保存する

### 自律移動
- `$roslaunch cuboid_nav navigation.launch map_file:=__/path/to/map-file__` 
- -> __/path/to/map-file__ を呼び出してその中で自律移動を行う
- -> 初期位置がずれている場合は、rvizメニューの`2D Pose Estimate`で自己位置の位置合わせを行う
- -> 自己位置がある程度合ったら、`2D Nav Goal` でゴール地点を指定し自律移動を行う
