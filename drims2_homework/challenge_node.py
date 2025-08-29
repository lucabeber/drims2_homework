import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
import numpy as np
from drims2_motion_server.motion_client import MotionClient
from drims2_msgs.srv import DiceIdentification
from moveit_msgs.msg import MoveItErrorCodes
import math
from sensor_msgs.msg import JointState
import time
HANDE_ACTION_NAME = '/gripper_action_controller/gripper_cmd'

home_joints = [1.57, -1.57, 1.57, -1.57, -1.57, 0.0]
eval_pos = [50, -74, 111, -37, -40, 0]
eval_pos = [math.radians(j) for j in eval_pos]

place_pos_rotate_nearing = [50, -75, 110, -35, -40, 0]
place_pos_rotate_nearing = [math.radians(j) for j in place_pos_rotate_nearing]
place_pos_rotate = [50, -55, 116, -61, -40, 0]
place_pos_rotate = [math.radians(j) for j in place_pos_rotate]

place_pos_nearing = [7, -109, -105, -56, 90, -167]
place_pos_nearing = [math.radians(j) for j in place_pos_nearing]
place_pos = [7, -116, -109, -45, 90, -167]
place_pos = [math.radians(j) for j in place_pos]

def scipy_rotation(initial_quaternion, axis, angle_degrees):
    initial_quat_array = np.array([
    initial_quaternion[0],
    initial_quaternion[1],
    initial_quaternion[2],
    initial_quaternion[3]
    ])
    initial_rotation = Rotation.from_quat(initial_quat_array)

    #pick_pose.pose.orientation.z -= math.pi 
#     pick_pose.pose.position.x = 0.0
#     pick_pose.pose.position.y = 0.0
#     pick_pose.pose.position.z = 0.0
    # pick_pose.pose.orientation.x = 0.0
    # pick_pose.pose.orientation.y = 1.0# logger.info("Attaching object...")
    # success = motion_client_node.attach_object("
    # pick_pose.pose.orientation.z = 0.0
    # pick_pose.pose.orientation.w = 1.0

    rotation_to_apply = Rotation.from_euler(axis, angle_degrees, degrees=True)
    final_rotation = rotation_to_apply * initial_rotation
    return final_rotation.as_quat()
def create_pose_stamped(frame_id, position, orientation, node):
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = frame_id
    pose_stamped.header.stamp = node.get_clock().now().to_msg()
    pose_stamped.pose.position.x = position[0]
    pose_stamped.pose.position.y = position[1]
    pose_stamped.pose.position.z = position[2]
    pose_stamped.pose.orientation.x = orientation[0]
    pose_stamped.pose.orientation.y = orientation[1]
    pose_stamped.pose.orientation.z = orientation[2]
    pose_stamped.pose.orientation.w = orientation[3]
    return pose_stamped

def pose_movement(node, logger, motion_client_node, pose_msg, pose_name, axis, rotation_deg):
    near_pose = PoseStamped()
    near_pose.header.frame_id = "checkerboard"
    near_pose.header.stamp = node.get_clock().now().to_msg()
    near_pose.pose = pose_msg.pose
    # near_pose.pose.position.x = pose_msg.pose.position.y
    # near_pose.pose.position.y = - pose_msg.pose.position.x
    near_pose.pose.position.x = 0.0
    near_pose.pose.position.y = 0.0
    near_pose.pose.position.z -= 0.05  # Approach from 5 cm above
    final_quat_list = [pose_msg.pose.orientation.x,
                       pose_msg.pose.orientation.y,
                       pose_msg.pose.orientation.z,
                       pose_msg.pose.orientation.w]
    logger.info(f"Quaternion: {final_quat_list}")
    if len(axis) > 0:
        for i in range(len(axis)):
            logger.info(f"Rotating around {axis[i]} by {rotation_deg[i]} degrees")
            final_quat_list = scipy_rotation(final_quat_list, axis[i], rotation_deg[i])
            logger.info(f"Quaternion: {final_quat_list}")

    logger.info(f"Quaternion: {final_quat_list}")
    near_pose.pose.orientation.x = final_quat_list[0]
    near_pose.pose.orientation.y = final_quat_list[1]
    near_pose.pose.orientation.z = final_quat_list[2]
    near_pose.pose.orientation.w = final_quat_list[3]
    logger.info(f"near pose orientation: {near_pose.pose.orientation}")
    logger.info(f"Moving to {pose_name}...")
    result = motion_client_node.move_to_pose(near_pose, cartesian_motion=True)
    # if result != MoveItErrorCodes.SUCCESS:
    #     logger.error(f"Failed to reach nearing position to {pose_name}: {result}, should be: {MoveItErrorCodes.SUCCESS}")
    #     return result
    
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "checkerboard"
    goal_pose.header.stamp = node.get_clock().now().to_msg()
    goal_pose.pose = near_pose.pose
    goal_pose.pose.position.z += 0.05 
    goal_pose.pose.orientation = near_pose.pose.orientation

    result = motion_client_node.move_to_pose(goal_pose, cartesian_motion=True)
    return result

def state_0_callback(vision_client_node, logger, desired_num, rot_cnt):
    num, dice_pose, success = vision_client_node.dice_identification()
    if success:    
        logger.error(f'Dice position {dice_pose.pose.position.x}, {dice_pose.pose.position.y}, {dice_pose.pose.position.z}')
        logger.error(f'Dice orientation {dice_pose.pose.orientation}')
        logger.error(f'Dice number {num}')
    if num == desired_num:
        logger.info(f'Detected desired dice number: {num}')
        new_state = 100
    elif num + desired_num == 7:
        new_state = 10
    elif rot_cnt == 1:
        new_state = 30
    else:
        new_state = 20

    return num, dice_pose,new_state, success

def state_10_callback(demo_node, logger, motion_client_node, vision_client_node, die_pose): 
    logger.info("Moving to pick pose...")
    pick_pose = die_pose
    # pick pose
    for i in range(2):
        result = pose_movement(demo_node, logger, motion_client_node, pick_pose, "pick pose", ['y'], [180])
        if result != MoveItErrorCodes.SUCCESS:
            logger.error(f"Failed to reach pick pose: {result}")
            return -1, result
        
        # close gripper and attach object
        logger.info("Closing gripper...")
        reached_goal, stalled = motion_client_node.gripper_command(position=0.20)  # 0.0 = closed
        logger.info(f"Gripper closed (reached_goal={reached_goal}, stalled={stalled})")

        logger.info("Attaching object...")
        success = motion_client_node.attach_object("dice", "tool0")  # "tool0" is the default TCP frame of UR10e
        logger.info(f"Attach success: {success}")

        # move to place position (rotate    )
        logger.info("Moving to rotate configuration...")
        result = motion_client_node.move_to_joint(place_pos_rotate_nearing)
        if result != MoveItErrorCodes.SUCCESS:
            logger.error(f"Failed to reach nearing rotate configuration: {result}")
            return -1, result
        result = motion_client_node.move_to_joint(place_pos_rotate)
        if result != MoveItErrorCodes.SUCCESS:
            logger.error(f"Failed to reach rotate configuration: {result}")
            return -1, result
        
        # open gripper and detach object
        logger.info("Opening gripper...")
        reached_goal, stalled = motion_client_node.gripper_command(position=0.045)  # 0.045 = open, adjust depending on gripper model
        logger.info(f"Gripper opened (reached_goal={reached_goal}, stalled={stalled})")

        logger.info("Detaching object...")
        success = motion_client_node.detach_object("dice")
        logger.info(f"Detach success: {success}")

        # move to safe position
        logger.info("Moving to nearing place configuration...")
        result = motion_client_node.move_to_joint(place_pos_nearing)
        if result != MoveItErrorCodes.SUCCESS:
            logger.error(f"Failed to reach nearing place configuration: {result}")
            return -1, result
        if i == 0:
            logger.info("Preparing for second flip...")
            num, pick_pose, success = vision_client_node.dice_identification()
            if not success:
                logger.error("Failed to identify dice.")
                return -1, None
    new_state = 0
    return new_state,result

def state_20_callback ( demo_node, logger, motion_client_node, vision_client_node, die_pose, desired_num):
    pick_pose = die_pose
    result = pose_movement(demo_node, logger, motion_client_node, pick_pose, "pick pose", ['y'], [180])
    if result != MoveItErrorCodes.SUCCESS:
        logger.error(f"Failed to reach pick pose: {result}")
        return -1, result
    
    # close gripper and attach object
    logger.info("Closing gripper...")
    reached_goal, stalled = motion_client_node.gripper_command(position=0.20)  # 0.0 = closed
    logger.info(f"Gripper closed (reached_goal={reached_goal}, stalled={stalled})")

    logger.info("Attaching object...")
    success = motion_client_node.attach_object("dice", "tool0")  # "tool0" is the default TCP frame of UR10e
    logger.info(f"Attach success: {success}")

    # move to place position (rotate    )
    logger.info("Moving to rotate configuration...")
    result = motion_client_node.move_to_joint(place_pos_rotate_nearing)
    if result != MoveItErrorCodes.SUCCESS:
        logger.error(f"Failed to reach nearing rotate configuration: {result}")
        return -1, result

    num, pick_pose, success = vision_client_node.dice_identification()
    if not success:
        logger.error("Failed to identify dice.")
        return -1, None
    if num == desired_num or num + desired_num == 7:
        return_value = 0
    else:
        return_value = 1

    result = motion_client_node.move_to_joint(place_pos_rotate)
    if result != MoveItErrorCodes.SUCCESS:
        logger.error(f"Failed to reach rotate configuration: {result}")
        return -1, result
    
    # open gripper and detach object
    logger.info("Opening gripper...")
    reached_goal, stalled = motion_client_node.gripper_command(position=0.045)  # 0.045 = open, adjust depending on gripper model
    logger.info(f"Gripper opened (reached_goal={reached_goal}, stalled={stalled})")

    logger.info("Detaching object...")
    success = motion_client_node.detach_object("dice")
    logger.info(f"Detach success: {success}")

    # move to safe position
    logger.info("Moving to nearing place configuration...")
    result = motion_client_node.move_to_joint(place_pos_nearing)
    if result != MoveItErrorCodes.SUCCESS:
        logger.error(f"Failed to reach nearing place configuration: {result}")
        return -1, result
    
    new_state = 0
    return return_value, new_state, result
def state_30_callback ( demo_node, logger, motion_client_node, vision_client_node, die_pose):
    pick_pose = die_pose
    result = pose_movement(demo_node, logger, motion_client_node, pick_pose, "pick pose", ['y', 'z'], [180,90])
    if result != MoveItErrorCodes.SUCCESS:
        logger.error(f"Failed to reach pick pose: {result}")
        return -1, result
    
    # close gripper and attach object
    logger.info("Closing gripper...")
    reached_goal, stalled = motion_client_node.gripper_command(position=0.20)  # 0.0 = closed
    logger.info(f"Gripper closed (reached_goal={reached_goal}, stalled={stalled})")

    logger.info("Attaching object...")
    success = motion_client_node.attach_object("dice", "tool0")  # "tool0" is the default TCP frame of UR10e
    logger.info(f"Attach success: {success}")

    # move to place position (rotate    )
    logger.info("Moving to rotate configuration...")
    result = motion_client_node.move_to_joint(place_pos_rotate_nearing)
    if result != MoveItErrorCodes.SUCCESS:
        logger.error(f"Failed to reach nearing rotate configuration: {result}")
        return -1, result

    result = motion_client_node.move_to_joint(place_pos_rotate)
    if result != MoveItErrorCodes.SUCCESS:
        logger.error(f"Failed to reach rotate configuration: {result}")
        return -1, result
    
    # open gripper and detach object
    logger.info("Opening gripper...")
    reached_goal, stalled = motion_client_node.gripper_command(position=0.045)  # 0.045 = open, adjust depending on gripper model
    logger.info(f"Gripper opened (reached_goal={reached_goal}, stalled={stalled})")

    logger.info("Detaching object...")
    success = motion_client_node.detach_object("dice")
    logger.info(f"Detach success: {success}")

    # move to safe position
    logger.info("Moving to nearing place configuration...")
    result = motion_client_node.move_to_joint(place_pos_nearing)
    if result != MoveItErrorCodes.SUCCESS:
        logger.error(f"Failed to reach nearing place configuration: {result}")
        return -1, result
    
    new_state = 0
    return new_state, result

def state_100_callback(node, logger, motion_client_node, home_joints):
    logger.info("Moving to home configuration...")
    result = motion_client_node.move_to_joint(home_joints)
    return result


class VisionClientNode(Node):
    def __init__(self):
        super().__init__('vision_client_node', use_global_arguments=False) 
        self.dice_identification_client = self.create_client(DiceIdentification, 
                                                             'dice_identification')

    def dice_identification(self):
        if not self.dice_identification_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("DiceIdentification service not available")

        request = DiceIdentification.Request()

        result_future = self.dice_identification_client.call_async(request)
        rclpy.spin_until_future_complete(self, result_future)
        
        face_number = result_future.result().face_number
        pose = result_future.result().pose
        success = result_future.result().success

        return face_number, pose, success



def main():
  
    rclpy.init()
    # MotionClient is already a Node, so we can use its logger directly
    motion_client_node = MotionClient(gripper_action_name=HANDE_ACTION_NAME)
    vision_client_node = VisionClientNode()
    demo_node          = rclpy.create_node(node_name="demo_node", use_global_arguments=False)
    joint_states_data = {}

    def joint_states_callback(msg):
        joint_states_data['names'] = msg.name
        joint_states_data['positions'] = msg.position

    # Create a temporary node to subscribe and reapose_named once
    def read_joint_states_once():
        sub = demo_node.create_subscription(JointState, '/joint_states', joint_states_callback, 10)
        # Wait for the first message
        while not joint_states_data:
            rclpy.spin_once(demo_node, timeout_sec=0.1)
        demo_node.destroy_subscription(sub)
   
    logger = demo_node.get_logger()

    # --- 0) Move to home configuration and open gripper---
    
    print(eval_pos)
    # logger.info("Moving to home configuration...")
    # result = motion_client_node.move_to_joint(home_joints)
    # logger.info(f"Home reached: {result}")
    # reached_goal, stalled = motion_client_node.gripper_command(position=0.045)  # Open gripper
    
    # if result != MoveItErrorCodes.SUCCESS:
    #     logger.error(f"Failed to reach home configuration: {result}")
    #     motion_client_node.destroy_node()
    #     vision_client_node.destroy_node()
    #     demo_node.destroy_node()
    #     rclpy.shutdown()
    #     return 0
    
    state = 0
    read_num = 0
    die_pose = PoseStamped()
    success = False
    result = MoveItErrorCodes.SUCCESS
    rot_cnt = 0

    # --------------------------------------------------------------------------------
    desired_num = 1
    # --------------------------------------------------------------------------------
    # while state != 100:
    #     # Dice identification
    #     if state == 0:
    #         logger.info("State 0: Starting die identification")
    #         read_num, die_pose, state, success = state_0_callback(vision_client_node, logger, desired_num, rot_cnt)
    #         if not success:
    #             logger.error('Dice identification failed')
    #             motion_client_node.destroy_node()
    #             vision_client_node.destroy_node()
    #             demo_node.destroy_node()
    #             rclpy.shutdown()
    #             return 0
    #     if state == 10:
    #         logger.info("State 10: flipping the die")
    #         state, result = state_10_callback(demo_node, logger, motion_client_node, vision_client_node, die_pose)
    #         if result != MoveItErrorCodes.SUCCESS:
    #             logger.error(f"Failed to reach home configuration: {result}")
    #             motion_client_node.destroy_node()
    #             vision_client_node.destroy_node()
    #             demo_node.destroy_node()
    #             rclpy.shutdown()
    #             return 0
    #         logger.info("Flip completed, back to die evaluation")
    #     if state == 20:
    #         logger.info("State 20: evaluating next rotations to do")
    #         rot_cnt, state, result = state_20_callback(demo_node, logger, motion_client_node, vision_client_node, die_pose, desired_num)
    #         if result != MoveItErrorCodes.SUCCESS:
    #             logger.error(f"Failed to reach home configuration: {result}")
    #             motion_client_node.destroy_node()
    #             vision_client_node.destroy_node()
    #             demo_node.destroy_node()
    #             rclpy.shutdown()
    #             return 0
    #         logger.info("evaluation completed, back to die evaluation")
    #     if state == 30:
    #         rot_cnt = 0
    #         state, result = state_30_callback(demo_node, logger, motion_client_node, vision_client_node, die_pose)
    #         if result != MoveItErrorCodes.SUCCESS:
    #             logger.error(f"Failed to reach home configuration: {result}")
    #             motion_client_node.destroy_node()
    #             vision_client_node.destroy_node()
    #             demo_node.destroy_node()
    #             rclpy.shutdown()
    #             return 0
    #         logger.info("rotation complete, back to evaluation")

    #     if state == 100:
    #         logger.info("State 100: going back home")
    #         result = state_100_callback(demo_node, logger, motion_client_node, home_joints)

    #         if result != MoveItErrorCodes.SUCCESS:
    #             logger.error(f"Failed to reach home configuration: {result}")
    #             motion_client_node.destroy_node()
    #             vision_client_node.destroy_node()
    #             demo_node.destroy_node()
    #             rclpy.shutdown()
    #             return 0
    #         logger.info("task completed successfully")
    # --- 1) Dice Identification
    reached_goal, stalled = motion_client_node.gripper_command(position=0.0)  # 0.0 = closed

    num, dice_pose, success = vision_client_node.dice_identification()
    if not success:
        logger.error('Dice identification failed')
        motion_client_node.destroy_node()
        vision_client_node.destroy_node()
        demo_node.destroy_node()
        rclpy.shutdown()
        return 0

    logger.error(f'Dice position {dice_pose.pose.position.x}, {dice_pose.pose.position.y}, {dice_pose.pose.position.z}')
    logger.error(f'Dice orientation {dice_pose.pose.orientation}')
    logger.error(f'Dice number {num}')

    result = pose_movement(demo_node, logger, motion_client_node, dice_pose, "pose 1", [], [])
    # if result != MoveItErrorCodes.SUCCESS:
    #     logger.error(f"Failed to reach pose 1: {result}")
    #     motion_client_node.destroy_node()
    #     vision_client_node.destroy_node()
    #     demo_node.destroy_node()
    #     rclpy.shutdown()
    #     return 0

    logger.info(f"Pose 1 reached: {result}")

    logger.info("Closing gripper...")
    reached_goal, stalled = motion_client_node.gripper_command(position=0.6)  # 0.0 = closed
    logger.info(f"Gripper closed (reached_goal={reached_goal}, stalled={stalled})")

    result = motion_client_node.move_to_joint(place_pos_nearing)
    result = motion_client_node.move_to_joint(place_pos)
    reached_goal, stalled = motion_client_node.gripper_command(position=0.0)  # 0.0 = closed

#     # logger.info("Attaching object...")
#     # success = motion_client_node.attach_object("dice", "tool0")  # "tool0" is the default TCP frame of UR10e
#     # logger.info(f"Attach success: {success}")

#     # logger.info("Moving to evaluation configuration...")
#     # result = motion_client_node.move_to_joint(eval_pos)
#     # logger.info(f"Configuration reached: {result}")

#     # num, dice_pose, success = vision_client_node.dice_identification()
#     # if not success:
#     #     logger.error('Dice identification failed')
#     #     motion_client_node.destroy_node()
#     #     vision_client_node.destroy_node()
#     #     demo_node.destroy_node()
#     #     rclpy.shutdown()
#     #     return 0

#     # logger.error(f'Dice position {dice_pose.pose.position.x}, {dice_pose.pose.position.y}, {dice_pose.pose.position.z}')
#     # logger.error(f'Dice orientation {dice_pose.pose.orientation}')
#     # logger.error(f'Dice number {num}')


#     logger.info("Moving to place configuration...")
#     result = motion_client_node.move_to_joint(place_pos_rotate)
#     logger.info(f"Configuration reached: {result}")

#     #   # --- 5) Open gripper + detach object ---
#     # logger.info("Opening gripper...")
#     # reached_goal, stalled = motion_client_node.gripper_command(position=0.045)  # 0.045 = open, adjust depending on gripper model
#     # logger.info(f"Gripper opened (reached_goal={reached_goal}, stalled={stalled})")

#     # logger.info("Detaching object...")
#     # success = motion_client_node.detach_object("dice")
#     # logger.info(f"Detach success: {success}")
#     # logger.info("Moving to home configuration...")
#     # result = motion_client_node.move_to_joint(home_joints)
#     # logger.info(f"Home reached: {result}")

#     # final_quat_array = scipy_rotation(dice_pose.pose.orientation, 'z', 90)
#     # pick_pose.pose.orientation.x = final_quat_array[0]
#     # pick_pose.pose.orientation.y = final_quat_array[1]
#     # pick_pose.pose.orientation.z = final_quat_array[2]
#     # pick_pose.pose.orientation.w = final_quat_array[3]

#     # logger.info("Moving to pick pose number 2...")
#     # result = motion_client_node.move_to_pose(pick_pose, cartesian_motion=True)
#     # if result != MoveItErrorCodes.SUCCESS:
#     #     logger.error(f"Failed to reach pick pose number 2: {result}")
#     #     motion_client_node.destroy_node()
#     #     vision_client_node.destroy_node()
#     #     demo_node.destroy_node()
#     #     rclpy.shutdown()
#     #     return 0

#     # logger.info(f"Pick pose number 2 reached: {result}")

    
#     # --- 3) Close gripper + attach object ---
#     # logger.info("Closing gripper...")
#     # reached_goal, stalled = motion_client_node.gripper_command(position=0.20)  # 0.0 = closed
#     # logger.info(f"Gripper closed (reached_goal={reached_goal}, stalled={stalled})")

#     # logger.info("Attaching object...")
#     # success = motion_client_node.attach_object("dice", "tool0")  # "tool0" is the default TCP frame of UR10e
#     # logger.info(f"Attach success: {success}")

#     # Create a copy of pick_pose for pick_pose_3
#     # pick_pose_3 = PoseStamped()
#     # pick_pose_3.header = pick_pose.header
#     # pick_pose_3.pose.position.x = pick_pose.pose.position.x
#     # pick_pose_3.pose.position.y = pick_pose.pose.position.y
#     # pick_pose_3.pose.position.z = pick_pose.pose.position.z
#     # pick_pose_3.pose.orientation.x = pick_pose.pose.orientation.x
#     # pick_pose_3.pose.orientation.y = pick_pose.pose.orientation.y
#     # pick_pose_3.pose.orientation.z = pick_pose.pose.orientation.z
#     # pick_pose_3.pose.orientation.w = pick_pose.pose.orientation.w
#     # pick_pose_3.pose.position.z += 0.2  # Lift the dice 20 cm
    
#     # logger.info("Moving to pick pose number 3...")
#     # result = motion_client_node.move_to_pose(pick_pose_3, cartesian_motion=True)
#     # if result != MoveItErrorCodes.SUCCESS:
#     #     logger.error(f"Failed to reach pick pose number 3: {result}")
#     #     motion_client_node.destroy_node()
#     #     vision_client_node.destroy_node()
#     #     demo_node.destroy_node()
#     #     rclpy.shutdown()
#     #     return 0

#     # logger.info(f"Pick pose number 3 reached: {result}")

#     # read_joint_states_once()
#     # current_joints = joint_states_data['positions']
#     # goal_joints = list(current_joints)[1:]
#     # # Shift elements 1, 2, 3 to the end of the list
#     # goal_joints = list(goal_joints)
#     # # Shift elements 1, 2, 3 to the end of the list
#     # goal_joints = goal_joints[1:] + goal_joints[0:1]
#     # goal_joints = goal_joints[::-1]
#     # # Convert all joint values from radians to degrees for logging
#     # goal_joints_deg = [joint * 180 / math.pi for joint in goal_joints]
#     # logger.info(f"goal joints (deg): {goal_joints_deg}")
#     # logger.info(f"goal_joint 2: {goal_joints[4]}")
#     # goal_joints[4] += math.pi/2  # Rotate joint 3 by 90 degrees
#     # logger.info(f"goal_joint 2: {goal_joints[4]}")
#     # goal_joints_deg = [joint * 180 / math.pi for joint in goal_joints]

#     # logger.info(f"goal joints: {goal_joints_deg}")
#     # logger.info(f"home joints type: {type(home_joints)}")
#     # logger.info(f"goal joints type: {type(goal_joints)}")

#     # # logger.info("Moving to rotate pose number 2...")
#     # # result = motion_client_node.move_to_joint(goal_joints)
#     # if result != MoveItErrorCodes.SUCCESS:
#     #     logger.error(f"Failed to reach rotate pose number 2: {result}")
#     #     motion_client_node.destroy_node()
#     #     vision_client_node.destroy_node()
#     #     demo_node.destroy_node()
#     #     rclpy.shutdown()
#     #     return 0

# #     logger.info(f"Rotate pose number 2 reached: {result}")
# #     time.sleep(0.5)
# #     place_pose = PoseStamped()
# #     place_pose.header.frame_id = "world"
# #     place_pose.header.stamp = demo_node.get_clock().now().to_msg()
# #     place_pose.pose.position.x = pick_pose.pose.position.x + 0.2
# #     place_pose.pose.position.y = pick_pose.pose.position.y + 0.2        
# #     place_pose.pose.position.z = pick_pose.pose.position.z
# #     place_pose.pose.orientation.x = pick_pose.pose.orientation.x
# #     place_pose.pose.orientation.y = pick_pose.pose.orientation.y
# #     place_pose.pose.orientation.z = pick_pose.pose.orientation.z
# #     place_pose.pose.orientation.w = pick_pose.pose.orientation.w
# #     logger.info("Moving to place pose...")

# #     result = motion_client_node.move_to_pose(place_pose, cartesian_motion=True)
# #     if result != MoveItErrorCodes.SUCCESS:
# #         logger.error(f"Failed to reach home configuration: {result}")
# #         motion_client_node.destroy_node()
# #         vision_client_node.destroy_node() 
# #         demo_node.destroy_node()
# #         rclpy.shutdown()
# #         return 0

# #     logger.info(f"Pick pose reached: {result}")
# # #     # --- 4) Move to another pose (place pose) ---
# #     place_pose = PoseStamped()
# #     place_pose.header.frame_id = "tip"
# #     place_pose.header.stamp = demo_node.get_clock().now().to_msg()
# #     place_pose.pose.position.x = 0.0
# #     place_pose.pose.position.y = 0.0
# #     place_pose.pose.position.z = -0.1
# #     place_pose.pose.orientation.x = 0.0
# #     place_pose.pose.orientation.y = 0.0
# #     place_pose.pose.orientation.z = 0.0
# #     place_pose.pose.orientation.w = 1.0

# #     logger.info("Moving to place pose...")
# #     result = motion_client_node.move_to_pose(place_pose, cartesian_motion=True)
# #     logger.info(f"Place pose reached: {result}")
# #     if result != MoveItErrorCodes.SUCCESS:
# #         logger.error(f"Failed to reach home configuration: {result}")
# #         motion_client_node.destroy_node()
# #         vision_client_node.destroy_node()
# #         demo_node.destroy_node()
# #         rclpy.shutdown()
# #         return 0

# #     # --- 5) Open gripper + detach object ---
#     # logger.info("Opening gripper...")
#     # reached_goal, stalled = motion_client_node.gripper_command(position=0.045)  # 0.045 = open, adjust depending on gripper model
#     # logger.info(f"Gripper opened (reached_goal={reached_goal}, stalled={stalled})")

#     # logger.info("Detaching object...")
#     # success = motion_client_node.detach_object("dice")
#     # logger.info(f"Detach success: {success}")

    motion_client_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
