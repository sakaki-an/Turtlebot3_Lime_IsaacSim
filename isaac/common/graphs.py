"""Action Graphs that later tutorials reuse.

Each function builds exactly the graph explained step by step in an earlier tutorial:
  * create_clock_graph()  - tutorial 06: publish /clock
  * create_drive_graph()  - tutorial 06: /cmd_vel -> Differential Controller -> wheels
  * create_odom_tf_joint_states_graph() - tutorial 07: /odom, TF odom->base_footprint, /joint_states
  * create_camera_graph()   - tutorial 08: RGB, depth, camera_info
  * create_lidar_graph()    - tutorial 09: /scan
  * create_imu_graph()      - tutorial 09: /imu
  * create_joint_command_graph() - tutorial 12: /isaac_joint_commands -> arm + gripper joints

Call them after SimulationApp is created and the ROS 2 bridge extension is enabled.
"""

from . import lime


def _og():
    import omni.graph.core as og

    return og


def create_clock_graph(graph_path: str = "/World/ROS2_Clock"):
    og = _og()
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            keys.SET_VALUES: [("PublishClock.inputs:topicName", "/clock")],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ],
        },
    )


def create_drive_graph(articulation_root: str, graph_path: str = "/World/ROS2_Drive", topic: str = "/cmd_vel"):
    import usdrt.Sdf

    og = _og()
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("BreakLinear", "omni.graph.nodes.BreakVector3"),
                ("BreakAngular", "omni.graph.nodes.BreakVector3"),
                ("DiffController", "isaacsim.robot.wheeled_robots.DifferentialController"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            keys.SET_VALUES: [
                ("SubscribeTwist.inputs:topicName", topic),
                ("DiffController.inputs:wheelRadius", lime.WHEEL_RADIUS),
                ("DiffController.inputs:wheelDistance", lime.WHEEL_BASE),
                ("DiffController.inputs:maxWheelSpeed", lime.MAX_WHEEL_SPEED),
                ("ArticulationController.inputs:targetPrim", [usdrt.Sdf.Path(articulation_root)]),
                ("ArticulationController.inputs:jointNames", lime.WHEEL_JOINTS),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                ("SubscribeTwist.outputs:execOut", "DiffController.inputs:execIn"),
                ("SubscribeTwist.outputs:linearVelocity", "BreakLinear.inputs:tuple"),
                ("BreakLinear.outputs:x", "DiffController.inputs:linearVelocity"),
                ("SubscribeTwist.outputs:angularVelocity", "BreakAngular.inputs:tuple"),
                ("BreakAngular.outputs:z", "DiffController.inputs:angularVelocity"),
                ("DiffController.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
            ],
        },
    )


# --8<-- [start:create_odom_tf_joint_states_graph]
def create_odom_tf_joint_states_graph(
    articulation_root: str,
    graph_path: str = "/World/ROS2_OdomTF",
    joint_states_topic: str = "/joint_states",
):
    import usdrt.Sdf

    og = _og()
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ComputeOdometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("PublishOdometry", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ("PublishOdomTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                ("ReadJointState", "isaacsim.sensors.physics.IsaacReadJointState"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ],
            keys.SET_VALUES: [
                ("ComputeOdometry.inputs:chassisPrim", [usdrt.Sdf.Path(articulation_root)]),
                ("PublishOdometry.inputs:topicName", "/odom"),
                ("PublishOdometry.inputs:odomFrameId", "odom"),
                ("PublishOdometry.inputs:chassisFrameId", "base_footprint"),
                ("PublishOdomTF.inputs:parentFrameId", "odom"),
                ("PublishOdomTF.inputs:childFrameId", "base_footprint"),
                ("ReadJointState.inputs:prim", [usdrt.Sdf.Path(articulation_root)]),
                ("PublishJointState.inputs:topicName", joint_states_topic),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ComputeOdometry.inputs:execIn"),
                ("ComputeOdometry.outputs:execOut", "PublishOdometry.inputs:execIn"),
                ("ComputeOdometry.outputs:execOut", "PublishOdomTF.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishOdometry.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishOdomTF.inputs:timeStamp"),
                ("ComputeOdometry.outputs:position", "PublishOdometry.inputs:position"),
                ("ComputeOdometry.outputs:orientation", "PublishOdometry.inputs:orientation"),
                ("ComputeOdometry.outputs:linearVelocity", "PublishOdometry.inputs:linearVelocity"),
                ("ComputeOdometry.outputs:angularVelocity", "PublishOdometry.inputs:angularVelocity"),
                ("ComputeOdometry.outputs:position", "PublishOdomTF.inputs:translation"),
                ("ComputeOdometry.outputs:orientation", "PublishOdomTF.inputs:rotation"),
                ("OnPlaybackTick.outputs:tick", "ReadJointState.inputs:execIn"),
                ("ReadJointState.outputs:execOut", "PublishJointState.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                ("ReadJointState.outputs:jointNames", "PublishJointState.inputs:jointNames"),
                ("ReadJointState.outputs:jointPositions", "PublishJointState.inputs:jointPositions"),
                ("ReadJointState.outputs:jointVelocities", "PublishJointState.inputs:jointVelocities"),
                ("ReadJointState.outputs:jointEfforts", "PublishJointState.inputs:jointEfforts"),
                ("ReadJointState.outputs:jointDofTypes", "PublishJointState.inputs:jointDofTypes"),
                ("ReadJointState.outputs:stageMetersPerUnit", "PublishJointState.inputs:stageMetersPerUnit"),
            ],
        },
    )
# --8<-- [end:create_odom_tf_joint_states_graph]


# --8<-- [start:create_joint_command_graph]
def create_joint_command_graph(
    articulation_root: str,
    graph_path: str = "/World/ROS2_JointCommands",
    topic: str = "/isaac_joint_commands",
):
    """Tutorial 12: apply sensor_msgs/JointState commands (from ros2_control) to the robot's joints."""
    import usdrt.Sdf

    og = _og()
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            keys.SET_VALUES: [
                ("SubscribeJointState.inputs:topicName", topic),
                ("ArticulationController.inputs:targetPrim", [usdrt.Sdf.Path(articulation_root)]),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                ("SubscribeJointState.outputs:execOut", "ArticulationController.inputs:execIn"),
                # The message says which joints it commands; pass the names through.
                ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
            ],
        },
    )
# --8<-- [end:create_joint_command_graph]


# --8<-- [start:create_camera_graph]
def create_camera_graph(
    camera_path: str,
    width: int = 640,
    height: int = 480,
    graph_path: str = "/World/ROS2_Camera",
    namespace: str = "/camera/camera",  # the topic names realsense2_camera uses on the real robot
    frame_id: str = "camera_color_optical_frame",
):
    """Tutorial 08: RGB image, depth image (aligned to RGB) and camera_info of one camera."""
    import usdrt.Sdf

    og = _og()
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("PublishRGB", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("PublishDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("PublishInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.SET_VALUES: [
                ("RenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(camera_path)]),
                ("RenderProduct.inputs:width", width),
                ("RenderProduct.inputs:height", height),
                ("PublishRGB.inputs:type", "rgb"),
                ("PublishRGB.inputs:topicName", f"{namespace}/color/image_raw"),
                ("PublishRGB.inputs:frameId", frame_id),
                ("PublishDepth.inputs:type", "depth"),  # float32 metres (32FC1)
                ("PublishDepth.inputs:topicName", f"{namespace}/aligned_depth_to_color/image_raw"),
                ("PublishDepth.inputs:frameId", frame_id),
                ("PublishInfo.inputs:topicName", f"{namespace}/color/camera_info"),
                ("PublishInfo.inputs:frameId", frame_id),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "RenderProduct.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "PublishRGB.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "PublishDepth.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "PublishInfo.inputs:execIn"),
                ("RenderProduct.outputs:renderProductPath", "PublishRGB.inputs:renderProductPath"),
                ("RenderProduct.outputs:renderProductPath", "PublishDepth.inputs:renderProductPath"),
                ("RenderProduct.outputs:renderProductPath", "PublishInfo.inputs:renderProductPath"),
            ],
        },
    )
# --8<-- [end:create_camera_graph]


# --8<-- [start:create_lidar_graph]
def create_lidar_graph(lidar_path: str, graph_path: str = "/World/ROS2_Lidar", topic: str = "/scan", frame_id: str = "base_scan"):
    """Tutorial 09: publish the RTX LiDAR as sensor_msgs/LaserScan."""
    import usdrt.Sdf

    og = _og()
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("PublishScan", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
            ],
            keys.SET_VALUES: [
                ("RenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(lidar_path)]),
                ("PublishScan.inputs:type", "laser_scan"),
                ("PublishScan.inputs:topicName", topic),
                ("PublishScan.inputs:frameId", frame_id),
                ("PublishScan.inputs:fullScan", True),  # one message per full 360 deg revolution
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "RenderProduct.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "PublishScan.inputs:execIn"),
                ("RenderProduct.outputs:renderProductPath", "PublishScan.inputs:renderProductPath"),
            ],
        },
    )
# --8<-- [end:create_lidar_graph]


# --8<-- [start:create_imu_graph]
def create_imu_graph(imu_path: str, graph_path: str = "/World/ROS2_Imu", topic: str = "/imu", frame_id: str = "imu_link"):
    """Tutorial 09: publish the physics IMU as sensor_msgs/Imu."""
    import usdrt.Sdf

    og = _og()
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ReadIMU", "isaacsim.sensors.physics.IsaacReadIMU"),
                ("PublishImu", "isaacsim.ros2.bridge.ROS2PublishImu"),
            ],
            keys.SET_VALUES: [
                ("ReadIMU.inputs:imuPrim", [usdrt.Sdf.Path(imu_path)]),
                ("ReadIMU.inputs:readGravity", True),  # a real IMU measures gravity too (+9.81 m/s^2 on z at rest)
                ("PublishImu.inputs:topicName", topic),
                ("PublishImu.inputs:frameId", frame_id),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ReadIMU.inputs:execIn"),
                ("ReadIMU.outputs:execOut", "PublishImu.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishImu.inputs:timeStamp"),
                ("ReadIMU.outputs:linAcc", "PublishImu.inputs:linearAcceleration"),
                ("ReadIMU.outputs:angVel", "PublishImu.inputs:angularVelocity"),
                ("ReadIMU.outputs:orientation", "PublishImu.inputs:orientation"),
            ],
        },
    )
# --8<-- [end:create_imu_graph]


def enable_ros2_realtime(simulation_app, hz: int = 60):
    """Enable the ROS 2 bridge and run the main loop in (approximately) real time (tutorial 06)."""
    import carb
    import isaacsim.core.experimental.utils.app as app_utils

    app_utils.enable_extension("isaacsim.ros2.bridge")
    settings = carb.settings.get_settings()
    settings.set("/app/runLoops/main/rateLimitEnabled", True)
    settings.set("/app/runLoops/main/rateLimitFrequency", hz)
    simulation_app.update()
