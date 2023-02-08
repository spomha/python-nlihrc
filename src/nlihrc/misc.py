"""Utility functions"""
from enum import Enum, unique

class GoalStatus(Enum):
    PENDING = 0   # The goal has yet to be processed by the action server
    ACTIVE  = 1   # The goal is currently being processed by the action server
    PREEMPTED  = 2   # The goal received a cancel request after it started executing
                     #   and has since completed its execution (Terminal State)
    SUCCEEDED = 3   # The goal was achieved successfully by the action server (Terminal State)
    ABORTED = 4   # The goal was aborted during execution by the action server due
                  #    to some failure (Terminal State)
    REJECTED = 5   # The goal was rejected by the action server without being processed,
                   #    because the goal was unattainable or invalid (Terminal State)
    PREEMPTING = 6   # The goal received a cancel request after it started executing
                     #    and has not yet completed execution
    RECALLING = 7   # The goal received a cancel request before it started executing,
                    #    but the action server has not yet confirmed that the goal is canceled
    RECALLED = 8   # The goal received a cancel request before it started executing
                   #    and was successfully cancelled (Terminal State)
    LOST = 9   # An action client can determine that a goal is LOST. This should not be
                #    sent over the wire by an action server


class CommandMode(Enum):
    CONTINUOUS = 0
    STEP = 1
    MODEL = 2


    #   0	& START ROBOT	& - \\
    #   1	& STOP ROBOT	& - \\
    #   2	& SET MODE STEP	& - \\
    #   3	& SET MODE CONTINUOUS	& - \\
    #   4	& SET MODE MODEL	& - \\
    #   5	& MOVE UP	& STEP/CONTINUOUS \\
    #   6	& MOVE DOWN	& STEP/CONTINUOUS \\
    #   7	& MOVE LEFT	& STEP/CONTINUOUS \\
    #   8	& MOVE RIGHT	& STEP/CONTINUOUS \\
    #   9	& MOVE BACK	& STEP/CONTINUOUS \\
    #   10	& MOVE FRONT	& STEP/CONTINUOUS \\
    #   11	& STOP EXECUTION	& STEP/CONTINUOUS/MODEL \\
    #   12   & STEP SIZE <VALUE> & STEP \\
    #   13   & OPEN TOOL & STEP/CONTINUOUS/MODEL \\
    #   14   & CLOSE TOOL & STEP/CONTINUOUS/MODEL \\
    #   15   & ROTATE TOOL <VALUE> & STEP/CONTINUOUS \\
    #   16   & SAVE POSITION <VALUE> & STEP/CONTINUOUS \\
    #   17   & LOAD POSITION <VALUE> & STEP/CONTINUOUS \\
    #   18   & HOME & STEP/CONTINUOUS/MODEL \\

@unique
class Command(Enum):
    START_ROBOT = 0
    STOP_ROBOT = 1
    SET_MODE_STEP = 2
    SET_MODE_CONTINUOUS = 3
    SET_MODE_MODEL = 4
    MOVE_UP = 5
    MOVE_DOWN = 6
    MOVE_LEFT = 7
    MOVE_RIGHT = 8
    MOVE_BACK = 9
    MOVE_FRONT = 10
    STOP_EXECUTION = 11
    STEP_SIZE = 12
    OPEN_TOOL = 13
    CLOSE_TOOL = 14
    ROTATE_TOOL = 15
    SAVE_POSITION = 16
    LOAD_POSITION = 17
    HOME = 18

class Controllers(Enum):
    MOVEIT = "/position_joint_trajectory_controller"
    SERVO = "/cartesian_controller"
