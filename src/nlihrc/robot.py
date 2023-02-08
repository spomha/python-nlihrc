"""Robot Manipulation Module"""

import rospy
import moveit_commander
import actionlib
from controller_manager_msgs.srv import SwitchController
import franka_gripper.msg
from actionlib_msgs.msg import GoalStatusArray
from moveit_msgs.msg import RobotTrajectory
import geometry_msgs.msg
from std_msgs.msg import String

from nlihrc.misc import GoalStatus, CommandMode, Command, Controller, MoveDirection, get_relative_orientation


class ControllerSwitcher:
    def __init__(self, active: Controller, stopped: Controller) -> None:
        """Initialize switch service"""
        self.active = active
        self.stopped = stopped
        self.strictness = 2
        self.start_asap = False
        self.timeout = 0.0

    def switch_controller(self, active: Controller, stop: Controller):
        if self.active == active:
            return
        rospy.wait_for_service('/controller_manager/switch_controller')
        try:
            switcher = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
            switcher([active.value], [stop.value], self.strictness, start_asap=self.start_asap, timeout=self.timeout)
            self.active = active
            self.stopped = stop
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)




class Manipulator:
    """Robot Manipulator"""
    def __init__(self, config) -> None:
        """Initialize manipulator"""
        self.config = config
        self.home_joints = self.config['robot']['home_joints']
        moveit_commander.roscpp_initialize([''])
        # initialize moveit commander
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("panda_arm")
        # Initialize servo controller publisher
        self.servo_pub = rospy.Publisher('/cartesian_controller/command', String, queue_size=1)
        # Set grasp tool as EE link
        self.move_group.set_end_effector_link("panda_hand_tcp")
        self.subscriber_feedback = rospy.Subscriber("/move_group/status",
            GoalStatusArray, self.moveit_status_callback,)
        self.move_group_status = GoalStatus.PENDING
        # Clients to send commands to the gripper
        self.grasp_action_client = actionlib.SimpleActionClient("/franka_gripper/grasp", franka_gripper.msg.GraspAction)
        self.move_action_client = actionlib.SimpleActionClient("/franka_gripper/move", franka_gripper.msg.MoveAction)
        # Transformation Matrices
        # Bring robot to home position during initialization
        self.home(wait=True)
        self.default_ee_pose = self.move_group.get_current_pose()

    def moveit_status_callback(self, data):
        """Callback function for topic /move_group/feedback"""
        # Only take the latest status
        if len(data.status_list) > 0:
            self.move_group_status = GoalStatus(data.status_list[-1].status)

    def moveit_home(self, wait=True):
        """Goto home position"""
        # Clear existing pose targets
        self.move_group.clear_pose_targets()
        # Plan home joint values
        self.move_group.set_joint_value_target(self.home_joints)
        plan = self.move_group.plan()
        self.moveit_execute_plan(plan, wait)

    def moveit_execute_plan(self, plan, wait=True) -> None:
        """Execute a given plan through move group"""
        rospy.loginfo(f"Current move group status: {self.move_group_status}")
        if self.move_group_status == GoalStatus.ACTIVE:
                rospy.logwarn("Robot busy, another trajectory is being executed. Wait for it to finish")
                return
        if isinstance(plan, RobotTrajectory):
            plan = [True, plan]
        if plan[0]:
            self.move_group.execute(plan[1], wait=wait)
        else:
            rospy.logwarn("Could not plan trajectory from current pose to home pose")
        
    def moveit_execute_cartesian_path(self, waypoints):
        """Execute cartesian path with some safety checks regarding pose waypoints"""
        z_min, z_max = 0.02, 0.50
        for pose in waypoints:
            if pose.position.z < z_min:
                rospy.logwarn(f"{pose.position.z = } is invalid. Using {z_min} instead")
                pose.position.z = z_min
            if pose.position.z > z_max:
                rospy.logwarn(f"{pose.position.z = } is invalid. Using {z_max} instead")
                pose.position.z = z_max
        plan, _ = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)  # jump_threshold
        self.moveit_execute_plan(plan)
    
    def open_gripper(self) -> None:
        """Open gripper"""
        goal = franka_gripper.msg.MoveGoal()
        goal.width = 0.08
        goal.speed = 0.1
        self.move_action_client.send_goal(goal)
        self.move_action_client.wait_for_result()

    def close_gripper(self):
        """Grasp object by closing gripper"""
        goal = franka_gripper.msg.GraspGoal()
        goal.width = 0.00
        goal.speed = 0.1
        goal.force = 5  # limits 0.01 - 50 N
        goal.epsilon = franka_gripper.msg.GraspEpsilon(inner=0.08, outer=0.08)
        self.grasp_action_client.send_goal(goal)
        self.grasp_action_client.wait_for_result()

    def servo_move(self, data):
        """Publish command to servo controller"""
        self.servo_pub.publish(data)

    

class CommandGenerator:
    """Command generator"""

    def __init__(self, config) -> None:
        """Initialize command generator"""
        self.config = config
        self.manipulator = Manipulator(self.config)
        self.mode = CommandMode.CONTINUOUS
        self.start_robot = False
        # Step size in meters
        self.step_size = 0.1
        # Commands that rely on numeric value use this parameter
        self.cmd_numeric_param = None
        # Controller switcher
        self.controller_switcher = ControllerSwitcher(active=Controller.MOVEIT, stopped=Controller.SERVO)

        # Saved positions
        self.saved_positions = {}

        self.cmds = {
            Command.START_ROBOT: lambda: self.setup_robot(True),
            Command.STOP_ROBOT: lambda: self.setup_robot(False),
            Command.SET_MODE_STEP: lambda: self.set_mode(CommandMode.STEP),
            Command.SET_MODE_CONTINUOUS: lambda: self.set_mode(CommandMode.CONTINUOUS),
            Command.SET_MODE_MODEL: lambda: self.set_mode(CommandMode.MODEL),
            Command.MOVE_UP: lambda: self.move(MoveDirection.UP),
            Command.MOVE_DOWN: lambda: self.move(MoveDirection.DOWN),
            Command.MOVE_LEFT: lambda: self.move(MoveDirection.LEFT),
            Command.MOVE_RIGHT: lambda: self.move(MoveDirection.RIGHT),
            Command.MOVE_FRONT: lambda: self.move(MoveDirection.FRONT),
            Command.MOVE_BACK: lambda: self.move(MoveDirection.BACK),
            Command.STOP_EXECUTION: lambda: self.stop_execution(),
            Command.STEP_SIZE: lambda: self.set_stepsize(),
            Command.OPEN_TOOL: lambda: self.oc_gripper(True),
            Command.CLOSE_TOOL: lambda: self.oc_gripper(False),
            Command.ROTATE_TOOL: lambda: self.rotate_gripper(),
            Command.SAVE_POSITION: lambda: self.save_position(),
            Command.LOAD_POSITION: lambda: self.load_position(),
            Command.HOME: lambda: self.home(),
        }

    def run(self, cmd, numeric=None):
        if self.start_robot is False and cmd != Command.START_ROBOT:
            rospy.logwarn(f"Command failed. Initialize robot with start command before specifying any other command!")
        rospy.loginfo(f"Running {cmd = }")
        self.cmd_numeric_param = numeric
        self.cmds[cmd]()

    def setup_robot(self, start):
        """Setup robot command"""
        self.start_robot = start

    def set_mode(self, mode):
        """Set mode command"""
        self.mode = mode

    def set_stepsize(self):
        """Step size command (given in centimeters)"""
        if self.cmd_numeric_param == None:
            return
        self.step_size = abs(self.cmd_numeric_param) / 100

    def oc_gripper(self, open):
        """Open/Close gripper"""
        if open:
            self.manipulator.open_gripper()
        else:
            self.manipulator.close_gripper()

    def rotate_gripper(self):
        """Rotate gripper by a given angle (in degree)"""
        if self.cmd_numeric_param == None:
            return
        ee_pose = self.manipulator.move_group.get_current_pose()
        ee_wxyz = [ee_pose.pose.orientation.w,
                   ee_pose.pose.orientation.x,
                   ee_pose.pose.orientation.y,
                   ee_pose.pose.orientation.z]
        new_wxyz = get_relative_orientation(ee_wxyz, self.cmd_numeric_param)
        
        pose = geometry_msgs.msg.Pose()
        pose.position.x = ee_pose.pose.position.x
        pose.position.y = ee_pose.pose.position.y
        pose.position.z = ee_pose.pose.position.z
        pose.orientation.w = new_wxyz[0]
        pose.orientation.x = new_wxyz[1]
        pose.orientation.y = new_wxyz[2]
        pose.orientation.z = new_wxyz[3]
        self.manipulator.moveit_execute_cartesian_path(pose)
    
    def move(self, direction):
        """Move commands"""
        if self.mode == CommandMode.STEP:
            step_size = self.step_size
        elif step_size == CommandMode.CONTINUOUS:
            step_size = 10
        else:
            return
        mapping = {
            MoveDirection.UP: f'Z,{step_size}',
            MoveDirection.DOWN: f'Z,-{step_size}',
            MoveDirection.LEFT: f'Y,-{step_size}',
            MoveDirection.RIGHT: f'Y,{step_size}',
            MoveDirection.FRONT: f'X,{step_size}',
            MoveDirection.BACK: f'X,-{step_size}',
        }
        move_param = mapping[direction]
        self.controller_switcher.switch_controller(active=Controller.SERVO, stop=Controller.MOVEIT)

        self.manipulator.servo_move(move_param)
    
    def stop_execution(self):
        """Stop running execution"""
        if self.controller_switcher.active == Controller.SERVO:
            self.manipulator.servo_move("Z,0")
        elif self.controller_switcher.active == Controller.MOVEIT:
            self.manipulator.move_group.stop()
            self.manipulator.move_group.clear_pose_targets()
        else:
            return

    def save_position(self):
        """Save position of end-effector pose"""
        if self.cmd_numeric_param == None:
            return
        self.saved_positions[self.cmd_numeric_param] = self.manipulator.move_group.get_current_pose()
    
    def load_position(self):
        """Load position of end-effector pose by executing cartesian trajectory"""
        if self.cmd_numeric_param == None or self.cmd_numeric_param not in self.saved_positions:
            return
        self.controller_switcher.switch_controller(Controller.MOVEIT, Controller.SERVO)
        self.manipulator.moveit_home(True)
        self.manipulator.moveit_execute_cartesian_path(self.saved_positions[self.cmd_numeric_param])

    def home(self):
        """Goto home joint values"""
        self.controller_switcher.switch_controller(Controller.MOVEIT, Controller.SERVO)
        self.manipulator.moveit_home(True)
