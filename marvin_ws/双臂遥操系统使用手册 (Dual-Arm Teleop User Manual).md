# 双臂遥操系统使用手册 \(Dual\-Arm Teleop User Manual\)

---

## 0\.  小臂每次上电前需要回到零点

按小臂上的红色记号摆放后，再上电。

否则上电后会出现上电后，小臂移动到错误位置。

![image\.png](图片和附件/image.png)

### 硬件连接与参数配置 \(Hardware Configuration\)

在启动节点之前，需要根据实际的硬件连接情况，配置大臂的 IP 地址以及小臂和夹爪的 USB 端口。

#### 大臂 IP 配置 \(Follower Arm IP\)

大臂通过以太网与上位机通信。默认 IP 为 `6.6.7.190`。
**配置方法**：

修改文件中的 `ip` 参数：

`/marvin_ws/install/share/robot_driver/launch/robot_servo_start_marvin.launch.py`

```xml
DeclareLaunchArgument('ip', default_value=EnvironmentVariable('ROBOT_IP', default_value='6.6.7.190')),
```

*\(或者在终端中临时设置环境变量：**`export ROBOT_IP=6.6.7.190`**\)*

#### 小臂 USB 端口配置 \(Leader Arm USB\)

小臂底层使用 Dynamixel 舵机，通过 USB 转 RS485 模块连接电脑。为了防止多次插拔导致端口号（如 `ttyUSB0`）漂移，系统采用设备的唯一 ID 进行绑定。
**获取设备 ID**：
插上小臂的 USB 后，在终端运行：

```bash
ls /dev/serial/by-id/
```

找到对应的设备名称（通常以 `usb-FTDI_USB__-__Serial_Converter...` 开头）。
**配置方法**：
分别修改左右臂的配置文件：

- **左小臂**：打开 `/marvin_ws/install/share/factr_teleop/configs/grav_comp_m6_left.yaml`，修改 `dynamixel_port` 字段。

- **右小臂**：打开 `/marvin_ws/install/share/factr_teleop/configs/grav_comp_m6_right.yaml`，修改 `dynamixel_port` 字段。

#### 夹爪 USB 端口配置 \(Gripper USB\)

夹爪通过 USB 转 TTL 模块连接电脑，通常被识别为 `/dev/ttyUSB*` 或 `/dev/ttyACM*`。
**获取端口号**：
插上夹爪的 USB 后，在终端运行 `ls /dev/ttyUSB*` 或 `ls /dev/ttyACM*` 查看新增的设备。
**配置方法**：
修改 `/marvin_ws/install/share/robot_driver/launch/robot_servo_start_marvin.launch.py` 文件中的夹爪参数：

```xml
DeclareLaunchArgument('gripper_port_left', default_value=EnvironmentVariable('ROBOT_GRIPPER_PORT_LEFT', default_value='/dev/ttyUSB0')),
DeclareLaunchArgument('gripper_port_right', default_value=EnvironmentVariable('ROBOT_GRIPPER_PORT_RIGHT', default_value='/dev/ttyUSB1')),
```

*\(或者在终端中临时设置环境变量：**`export ROBOT_GRIPPER_PORT_LEFT=/dev/ttyUSB0`**\)*

#### 如果使用docker请将上述硬件端口参数配置到run\_marvin\_m6\.sh中：



### 遥操臂的使用 \(Using the Teleop Arms\)

⚠️ **安全警告 \(Safety Notice\)**

- **解锁/杀掉小臂程序前请双手握住小臂手柄**，避免主手和从手发生意外掉落。

- **请确保主手之间保持适当的安全距离**，并在锁定到安全位置后再进行同步操作，以防出现意外掉落或碰撞等异常情况。

- **锁定后请勿碰撞主手**，避免舵机受到外力损坏。

- **操作时请注意**，避免两个从手之间发生相互干涉，导致机械臂或相机损坏。

#### 启动docker 

```Plain Text
#通过 run_marvin_m6.sh <image_name>启动image。
run_marvin_m6.sh <image_name>
```

#### 节点启动 \(Starting the Nodes\)

对于双臂遥操系统，需要分别启动大臂（机器人本体及夹爪）和同构小臂（遥操主端）：

**步骤 A: 启动双臂本体与夹爪 \(Follower Arms \& Grippers\)**

```Plain Text
# 查看容器名称
docker ps
#进入刚创建的docker container
docker exec -it <your container name> bash
```

```Plain Text
## 每个容器终端内设置相同 ROS2 DDS 参数：
unset ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID=42
```

```bash
source /marvin_ws/install/setup.bash
ros2 launch robot_driver robot_servo_start_marvin.launch.py ip:=${ROBOT_IP}
```

```Go
ros2 launch robot_driver robot_servo_start_marvin.launch.py \
  ip:=${ROBOT_IP} \
  use_left_gripper:=true \
  use_right_gripper:=true \
  gripper_port_left:=${ROBOT_GRIPPER_PORT_LEFT} \
  gripper_port_right:=${ROBOT_GRIPPER_PORT_RIGHT}
```

该 launch 会同时启动左右两台大臂的伺服节点，以及配置好的末端硬件夹爪\(目前launch里未加入gripper，\*需要自行配置。\)。

**步骤 B: 启动双端遥操小臂 \(Leader Arms\)**

```Plain Text
#再新建一个terminal进入刚创建的docker container
docker exec -it <your container name> bash
```

```Plain Text
## 每个容器终端内设置相同 ROS2 DDS 参数：
unset ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID=42
```

```bash
source /marvin_ws/install/setup.bash
#第一次启动的时候会报错，需要输入
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
#建立双臂
ros2 launch factr_teleop start_teleop_m6_dual.launch.py use_keyboard:=true
#单left臂
ros2 launch factr_teleop start_teleop_m6_left.launch.py use_keyboard:=true
#单right臂
ros2 launch factr_teleop start_teleop_m6_right.launch.py use_keyboard:=true


```

该 launch 会拉起左右两个小臂的底层通讯节点，并同时启动键盘事件监听以管理模式切换。

#### 使用键盘控制位姿同步与遥操 \(Keyboard Control for Sync \& Teleop\)

在上述 小臂启动后，后台会自动运行 `keyboard_gripper.py` 节点，监听全局键盘输入（不需要将焦点放在终端窗口上）。

系统启动后，默认处于 **IDLE \(空闲\)** 模式。此时小臂和大臂的姿态可能不一致，**切勿直接开始遥操**，否则可能会引起大臂突变。

正确的开始遥操流程如下：

1. **按下数字键 ****`1`**** （启动位姿同步）**

    - 触发 `TELEOP_SYNCING` 状态。

    - 此时，小臂（主端）将**自动平滑移动**，去主动对齐大臂（从端）当前的位置。

    - 等待几秒钟，直到小臂移动到位，此时系统会自动进入 `SYNCED` 状态。小臂会像钉在半空中一样**保持不动**。

2. **按下数字键 ****`2`**** （进入遥操控制）**

    - 触发 `TELEOP` 状态。

    - 此时，小臂的“位置保持”锁定解除，进入自由浮动状态。

    - 小臂开始实时控制大臂，此时你可以通过操作小臂来移动大臂。

3. **按下数字键 3 （脱离遥操控制）**

    - 大臂脱离小臂控制。

*注意：按下 **`q`** 键可退出键盘监听节点。*

\#\#如果工控机没有 X11 桌面环境，或容器无法获取全局键盘事件：

```Plain Text
```bash
ros2 launch factr_teleop start_teleop_m6_dual.launch.py use_keyboard:=false
用 topic 发送模式指令
ros2 topic pub -1 /mode/switch_sync std_msgs/msg/String "{data: 'switch_sync'}"
ros2 topic pub -1 /mode/switch_teleop std_msgs/msg/String "{data: 'switch_teleop'}"
ros2 topic pub -1 /mode/switch_stop std_msgs/msg/String "{data: 'switch_stop'}"
```

### 高级进阶 \(Advanced Topics\)

#### 示例：如何从大臂读取状态 \(Reading State from Follower Arm\)

大臂的状态默认高频发布在 `/left_joint_state` 和 `/right_joint_state` 话题上，消息类型为标准的 `sensor_msgs/JointState`。

在终端快速查看：

```bash
rostopic echo /left_joint_state
```

在代码中读取（Python示例）：

```python
import rospy
from sensor_msgs.msg import JointState

def arm_state_callback(msg):
    # msg.position 包含 6 个元素，分别代表关节 1 到关节 6 的当前角度（单位：弧度）
    # 限制：受机械臂物理限位影响（如 Nova2 在 -2π 到 2π 之间）
    positions = msg.position  

    # msg.velocity 包含 6 个元素，分别代表关节 1 到关节 6 的当前角速度（单位：rad/s）
    # 限制：受电机最高转速影响，通常在 1.57 到 3.14 rad/s 之间
    velocities = msg.velocity 

    # msg.effort 包含 6 个元素，分别代表关节 1 到关节 6 当前的电机力矩/电流（关键反馈！）
    # 限制：受电机堵转电流/最大扭矩限制。
    efforts = msg.effort      

rospy.Subscriber('/left_joint_state', JointState, arm_state_callback)
```



#### 示例：如何更换新的夹爪 \(How to Change the Gripper\)

系统中的夹爪节点通常配置在大臂启动的 `launch` 文件中。由于系统是完全解耦的，**你不需要修改任何小臂端的代码** 就可以让小臂的 trigger（扳机）控制新夹爪并获得力反馈。

小臂的 trigger 位置会直接发布给物理夹爪。因此你只需要在**大臂端**完成替换：

**步骤 1：编写新夹爪的 ROS 驱动节点**
在 `robot_driver/script` 目录下新建一个 Python 驱动脚本（例如 `new_gripper.py`），它必须遵守两个核心 Topic 接口：

- **订阅**`/{side}_gripper/ctrl`：接收目标位置（`msg.position[0]`），并转换为硬件控制指令发给新夹爪。

- **发布**`/{side}_gripper/state`：以较高频率（建议 30Hz \~ 50Hz）发布夹爪真实位置和**电流/受力**（`msg.effort[0]`）。（这个 effort 值会被传回小臂，转化为阻力感。发布频率通常 30Hz 以上就能保证手感流畅）。

*驱动模板示例：*

```python
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

class NewGripperDriver:
    def __init__(self):
        rospy.init_node('new_gripper_node')
        self.sub = rospy.Subscriber('gripper/ctrl', JointState, self.ctrl_callback)
        self.pub = rospy.Publisher('gripper/state', JointState, queue_size=1)
        # 初始化硬件: self.hardware = YourHardwareDevice()

    def ctrl_callback(self, msg):
        target_pos = msg.position[0]
        # TODO: 下发 target_pos 到夹爪硬件

    def publish_state(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            state_msg = JointState()
            state_msg.header.stamp = rospy.Time.now()
            # TODO: 从硬件读取真实位置和受力(力反馈关键)
            state_msg.position = [0.0] # 填入实际位置
            state_msg.effort = [0.0]   # 填入实际电流/力
            self.pub.publish(state_msg)
            rate.sleep()

if __name__ == '__main__':
    driver = NewGripperDriver()
    driver.publish_state()
```

**步骤 2：修改大臂启动的 Launch 文件**
在 `robot_servo_start_marvin.launch` 中，注释掉原有的夹爪节点，换成你的新节点，并配置好 `remap`：

```xml
<!-- 增加你的新夹爪节点 -->
    <node pkg="robot_driver" type="new_gripper.py" name="new_gripper" output="screen">
        <param name="port" value="/dev/ttyUSB0" />
        <!-- 将内部话题 remap 到左/右臂的专属话题 -->
        <remap from="gripper/ctrl" to="/left_gripper/ctrl"/>
        <remap from="gripper/state" to="/left_gripper/state"/>
    </node>
```

重启大臂 launch 后，小臂的 trigger 就可以无缝控制新夹爪并感受力反馈了。

#### 示例：如何通过代码让小臂自动移动 \(Auto\-moving the Leader Arm\)

系统内置了平滑轨迹规划接口，允许客户通过代码直接控制小臂（主端）移动到指定位姿并保持。

**步骤 1：确保小臂处于位置保持状态**
在发送移动指令前，小臂必须处于 **SYNCED（已同步/位置保持）** 状态。你可以通过代码向 `/mode/switch_sync` 发送任意字符串来触发。

**步骤 2：发布目标位姿**
向 `/left_leader_arm/target_joint_state` \(或 right\) 发布包含 **7个元素**（6个机械臂关节角度 \+ 1个夹爪开合度）的 `JointState` 消息。

*Python 示例代码：*

```python
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

def move_leader_arm_auto():
    rospy.init_node('leader_arm_auto_mover')
    left_target_pub = rospy.Publisher('/left_leader_arm/target_joint_state', JointState, queue_size=1)
    mode_pub = rospy.Publisher('/mode/switch_sync', String, queue_size=1)
    rospy.sleep(1.0)

    # 1. 触发 SYNCED 模式，让小臂进入“位置保持”状态
    rospy.loginfo("进入位置保持模式...")
    mode_pub.publish(String(data="sync"))
    rospy.sleep(2.0)

    # 2. 构建目标位姿消息 (6个臂关节 + 1个夹爪)
    target_msg = JointState()
    target_msg.header.stamp = rospy.Time.now()
    target_msg.position = [0.0, -0.5, 1.0, 0.0, 1.5, 0.0, 0.0] 

    # 3. 发送指令，小臂将自动平滑移动
    rospy.loginfo("发送目标位姿，小臂开始自动平滑移动...")
    left_target_pub.publish(target_msg)

if __name__ == '__main__':
    move_leader_arm_auto()
```

### 节点 List 与核心 Topics \(Nodes \& Topics List\)

#### 核心节点列表 \(Core Nodes\)

在系统中，双臂控制主要由以下核心节点协同工作完成：

- `robot_servo_driver`：**大臂本体驱动节点**，负责与真实的从端大臂（如 Marvin M6）进行底层通讯，读取大臂角度和发号施令。

- `factr_teleop`：**同构小臂遥操主节点**（左臂/右臂各一个），负责读取遥操主端的位置与力度，并控制小臂跟随。

- `keyboard_gripper`：**全局键盘监听节点**，负责捕捉按键（1, 2, 3），下发模式切换指令。

- `gripper_control.py` \(或 `new_gripper.py`\)：**物理夹爪驱动节点**，独立于大臂运行，负责转换夹爪的开合度和电流反馈。\*需要自行配置。

#### 核心话题流转逻辑 \(Core Topics\)

基于 `sensor_msgs/JointState` 和自定义指令，目前系统纯遥操的核心话题流转逻辑非常简洁（以 `left` 左臂为例，`right` 右臂同理）：



|Topic 名称|消息类型|用途|发布方|订阅方|
|---|---|---|---|---|
|`/left_joint_state`|`JointState`|**大臂当前状态**：包含大臂各关节的 position, velocity, effort \(力矩\)|`robot_driver` \(大臂本体\)|`factr_teleop` \(用于同步\)|
|`/left_joint_control`|`JointState`|**大臂控制指令**：遥操小臂发送给大臂的目标动作位置|`factr_teleop` \(遥操节点\)|`robot_driver` \(大臂本体\)|
|`/leader_arm/target_joint_state`<br>|`JointState`|**小臂自动移动目标**：发送包含7个元素\(6关节\+1夹爪\)的位姿，小臂会自动平滑移动到该位置|外部代码/客户脚本，|`factr_teleop` \(用于程序控制小臂\)<br>|
|`/left_teleop_gripper/ctrl`<br>|`JointState`|**夹爪控制指令**：由小臂末端 trigger 发出的目标夹取度|`factr_teleop`|物理夹爪驱动节点|
|`/left_gripper/state`|`JointState`|**夹爪状态反馈**：当前夹爪的位置及电机受力|物理夹爪驱动节点|`factr_teleop` \(用于夹爪阻力感\)|
|`/mode/switch_sync`|`std_msgs/String`|**模式指令（键盘1）**：触发小臂主动同步大臂位置，或进入位置保持模式|键盘节点/外部代码|`factr_teleop`|
|`/mode/switch_teleop`<br>|`std_msgs/String`|**模式指令（键盘2）**：进入遥操控制大臂模式|键盘节点|`factr_teleop`|
|`/left_end_pose`|`geometry_msgs/PoseStamped`|**大臂末端位姿**：大臂当前末端的笛卡尔空间位姿|`robot_driver` \(大臂本体\)|外部节点|
|`/left_end_force`|`geometry_msgs/WrenchStamped`|**大臂末端受力**：大臂末端的六维力/力矩反馈|`robot_driver` \(大臂本体\)|外部节点|


