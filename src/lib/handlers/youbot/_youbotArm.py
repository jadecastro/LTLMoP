import sys, math, time
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

class YouBotArm:

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander("arm")

    def moveUp(self):
        print "Moving the arm up..."

        self.group.clear_pose_targets()

        group_variable_values = group.get_current_joint_values()
        print "============ Joint values: ", group_variable_values

        group_variable_values[0] = 1.0
        group.set_joint_value_target(group_variable_values)

        plan2 = group.plan()
        
        group.go(wait=True)


if __name__ == "__main__":

    s = YouBotArm()

    s.moveUp()

    moveit_commander.roscpp_shutdown()

