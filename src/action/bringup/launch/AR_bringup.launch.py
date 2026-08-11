#!/usr/bin/env python3
#
# AR_bringup.launch.py
# 一键启动 Piper 机械臂手眼标定所需的所有节点。
#
# 启动内容：
#   1. Realsense D435i 相机驱动
#   2. Piper 机械臂 CAN 控制器（需 can0 正常且臂上电）
#   3. robot_state_publisher + joint_state_publisher_gui（滑块控制）
#   4. 静态 TF: link6 → camera_link（手眼标定初始估计）
#   5. rviz2（显示 Camera 图像 + TF 标记帧，无 RobotModel）
#
# 使用方式：
#   ros2 launch bringup AR_bringup.launch.py
#
# 机械臂启动前准备：
#   1. CAN 接口： sudo ip link set can0 up type can bitrate 1000000
#   2. 确认机械臂已上电且 CAN 线已连接
#   3. 确认 /dev/video* 设备节点存在（Realsense 相机已连接）
#
# 控制流程（2026-08-04 修复 GUI 死循环问题）：
#   启动 → arm 控制器使能 → 4s 后调用 /move_home 回初始位置(全0)
#   → 10s 后 joint_state_publisher_gui 启动，滑块开始控制
#   → 滑块值 → control/joint_states → arm 控制器 move_j 执行
#   → robot_state_publisher 读取 feedback/joint_states，TF 跟随实际运动
#
# 注意：joint_state_publisher_gui 不能加 source_list，
#   否则滑块值会被 200Hz 反馈覆盖，形成控制死循环（GUI 失灵）。
#

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    # ============================================================
    # 1. Realsense D435i 相机驱动
    # ============================================================
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch', 'rs_launch.py',
            ])
        ]),
        launch_arguments={
            'camera_name': 'camera',
            'camera_namespace': 'camera',
            'pointcloud.enable': 'false',
            'depth_module.depth_profile': '640x480x30',
            'rgb_camera.color_profile': '640x480x30',
            'enable_sync': 'true',
            'align_depth.enable': 'false',
            'decimation_filter.enable': 'true',
        }.items(),
    )

    # ============================================================
    # 2. Piper 机械臂 CAN 控制器
    #
    # 参考参数（来自 start_single_agx_arm.launch.py）：
    #   arm_type:          piper / nero / piper_h / piper_l / piper_x
    #   can_port:          CAN 接口名，默认 can0
    #   auto_enable:       启动时自动使能关节（默认 true）
    #   enable_timeout:    使能/固件等待超时秒数（默认 5.0）
    #   fast_mode:         关节控制走 move_js(MIT) 模式（默认 false）
    #   speed_percent:     运动速度百分比（默认 100）
    #   effector_type:     none / agx_gripper / revo2 / revo2_touch
    #   tcp_offset:        TCP 偏移 [x,y,z,roll,pitch,yaw]
    #   control_enabled:   是否接受外部控制指令（默认 true）
    #   pub_rate:          反馈发布频率 Hz（默认 200）
    #
    # 注意：控制器初始化时会读固件版本，
    # 如果 CAN 不通或机械臂未上电，进程会 exit(1) 退出。
    # ============================================================
    arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('agx_arm_ctrl'),
                'launch', 'start_single_agx_arm.launch.py',
            ])
        ]),
        launch_arguments={
            'arm_type': 'piper',
            'can_port': 'can0',
        }.items(),
    )

    # ============================================================
    # 2.5 机械臂回初始位置
    #     原实现: 启动后 4s 调用 /move_home (move_j 全 0)。
    #     现改为: 启动后 4s 调用 /control/move_j 停在"当前标定姿态"
    #     (J6≈61.6°=1.0749rad, 末端正对前方), 不再回全零 home。
    #     若机械臂不在该姿态, 请先手动调到位或用滑块调整后更新下方关节角。
    # ============================================================
    move_home_action = ExecuteProcess(
        cmd=['bash', '-c',
             'sleep 4 && ros2 topic pub -1 /control/move_j sensor_msgs/msg/JointState '
             '"{header: {stamp: now}, name: [joint1, joint2, joint3, joint4, joint5, joint6], '
             'position: [0.0144, 0.0, 0.0, 0.0, 0.0, 1.0749], velocity: [], effort: []}"'],
        output='screen',
    )

    # ============================================================
    # 3. robot_state_publisher + joint_state_publisher_gui
    #    加载完整底盘+机械臂 URDF，发布 TF 链
    #
    #    robot_state_publisher 读取 feedback/joint_states(真实反馈),
    #    使 base_link→link6 跟随机械臂实际运动
    #
    #    joint_state_publisher_gui 滑块 → control/joint_states
    #    → arm 控制器 move_j 执行（不能加 source_list, 否则死循环）
    # ============================================================
    model_path = '/home/ros/MC_ARM_baseline_nav2_ok/src/driver/car_sim/urdf/car_with_piper.urdf.xacro'
    robot_description = ParameterValue(
        Command(['xacro ', model_path]),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        # 读取合并关节状态: 臂关节=feedback, 轮子/夹爪=0, TF 完整
        remappings=[('joint_states', 'merged_joint_states')],
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        # 注意: 不能加 source_list, 否则滑块值会被 200Hz 反馈覆盖
        parameters=[
            {'rate': 200},
            # 滑块初始值 = 当前标定姿态(正对前方), 而非全零 home。
            # GUI 只读取 zeros 参数作为滑块初始值(内部参数名, 不是 initial_positions)
            {'zeros': {'joint1': 0.0144, 'joint2': 0.0,
                       'joint3': 0.0,   'joint4': 0.0,
                       'joint5': 0.0,   'joint6': 1.0749}},
        ],
        remappings=[('joint_states', 'control/joint_states')],
    )

    # GUI 延迟启动: 等机械臂先完成回初始位置(home)
    delayed_gui_node = TimerAction(period=10.0, actions=[joint_state_publisher_gui_node])
    # 关节状态合并: 臂关节(joint1-6)来自真实反馈, 轮子/夹爪保持默认0
    # 解决 robot_state_publisher 缺少轮子/夹爪关节状态导致 TF 缺失的问题
    joint_state_merger_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[
            {'rate': 200},
            {'source_list': ['feedback/joint_states']},
        ],
        remappings=[('joint_states', 'merged_joint_states')],
    )



    # ============================================================
    # 3.5 世界坐标系: world(地面) → BODY
    #    BODY 是 URDF 根 link。轮心相对 BODY z=-0.075972, 轮子半径 0.0769,
    #    故 BODY 原点应在 world z = 0.075972 + 0.0769 = 0.152872, 轮子压地。
    #    (不能用 world→base_link: 那会让 base_link 有 world 和 BODY 两个
    #     parent 形成冲突; 应让 URDF 根 BODY 挂到 world, 子树全部连通)
    # ============================================================
    world_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0.152872',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'world', '--child-frame-id', 'BODY',
        ],
    )

    # ============================================================
    # 3.6 补充 URDF 中未被 robot_state_publisher 发布的固定 TF
    #     原因: BODY 是带 inertia 的 root link, 触发 KDL 警告导致 rsp
    #     不发布任何 fixed joint 变换, 部分固定链断连。
    #     这里手动补上 rsp 缺失的固定 TF(与 URDF origin 一致):
    #       mid360              BODY → mid_Link      xyz=0.1325 0 0.010499  yaw=π
    #       flange_joint        link6 → flange_link  xyz=0 0 0
    #       gripper_base_joint  flange_link → gripper_base  xyz=0 0 0.0045
    #     注意: body_to_base_link(BODY→base_link) 不补——URDF 树 BODY 是根,
    #     base_link 的 parent 已是 BODY(world→BODY 挂根后自动连通),
    #     再补 BODY→base_link 会让 base_link 出现双 parent 冲突。
    # ============================================================
    mid360_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0.1325', '--y', '0', '--z', '0.010499',
            '--roll', '0', '--pitch', '0', '--yaw', '3.1416',
            '--frame-id', 'BODY', '--child-frame-id', 'mid_Link',
        ],
    )
    flange_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'link6', '--child-frame-id', 'flange_link',
        ],
    )
    gripper_base_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0.0045',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'flange_link', '--child-frame-id', 'gripper_base',
        ],
    )

    # ============================================================
    # 4. 静态 TF: link6 → camera_link (手眼标定结果)
    #    标定日期: 2026-07-30  算法: Tsai-Lenz (OpenCV)
    #    标定数据: 171 组  Tag ID=12, size=0.128m
    #
    #    说明: 标定矩阵是 OpenCV 相机系(link6_T_camera, +Z=光轴),
    #    即 optical 系。落地为 link6→camera_link 时需补偿旋转:
    #    camera_link 光轴是 +X(Realsense 约定), camera_link→optical
    #    内部链由相机进程发布。以下数值 = 标定矩阵 × 该内部链逆,
    #    使 camera_link 的 +X 对齐光轴方向, TF 树唯一无冲突。
    # ============================================================
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '-0.038497', '--y', '0.101017', '--z', '0.004873',
            '--roll', '0.079287', '--pitch', '-1.175054', '--yaw', '-1.026760',
            '--frame-id', 'link6', '--child-frame-id', 'camera_link',
        ],
    )

    # ============================================================
    # 5. rviz2（显示 Camera 图像 + TF 标记帧，无 RobotModel）
    # ============================================================
    rviz_config = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'ar_calib.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        # 1
        camera_launch,
        # 2
        arm_launch,
        # 2.5 机械臂回初始位置
        move_home_action,
        # 3.5 world → BODY
        world_tf_node,
        # 3.6 补充 rsp 未发布的固定 TF (mid360/flange/gripper_base)
        mid360_tf_node,
        flange_tf_node,
        gripper_base_tf_node,
        # 3
        joint_state_merger_node,
        robot_state_publisher_node,
        delayed_gui_node,
        # 4
        static_tf_node,
        # 5 rviz: Camera image + TF marker, no RobotModel
        rviz_node,
    ])
