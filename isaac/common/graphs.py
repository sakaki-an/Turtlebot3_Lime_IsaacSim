"""Action Graphs that later tutorials reuse.

Each function builds exactly the graph explained step by step in an earlier tutorial:
  * create_clock_graph()  - tutorial 06: publish /clock
  * create_drive_graph()  - tutorial 06: /cmd_vel -> Differential Controller -> wheels
  * create_odom_tf_joint_states_graph() - tutorial 07: /odom, TF odom->base_footprint, /joint_states

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


def enable_ros2_realtime(simulation_app, hz: int = 60):
    """Enable the ROS 2 bridge and run the main loop in (approximately) real time (tutorial 06)."""
    import carb
    import isaacsim.core.experimental.utils.app as app_utils

    app_utils.enable_extension("isaacsim.ros2.bridge")
    settings = carb.settings.get_settings()
    settings.set("/app/runLoops/main/rateLimitEnabled", True)
    settings.set("/app/runLoops/main/rateLimitFrequency", hz)
    simulation_app.update()
