import time
import rclpy
import sys
import signal
import os
from message_filters import ApproximateTimeSynchronizer,Subscriber
# pypkg_path = os.path.join(os.path.expanduser('~'),"anaconda3/envs/")
# sys.path.append("/home/dc/anaconda3/envs/dc/lib/python3.8/site-packages")
import numpy as np
import cv2
import h5py
from cv_bridge import CvBridge
from pynput import keyboard

import rclpy.logging
import rclpy.publisher
from rclpy.node import Node
from sensor_msgs.msg import Image
from arm_control.msg import PosCmd
from arx5_arm_msg.msg import RobotCmd
from arx5_arm_msg.msg import RobotStatus
import os

def count_files_with_extension(directory, extension):
    count = 0
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith(extension):
                count += 1
    return count

global data_dict, step, Max_step, dataset_path, finish_recording

# parameters
step = 0
Max_step = 2000
finish_recording = False
directory_path = f'/home/philaptop/wfy/dp_data'
extension = '.hdf5' 
episode_idx = count_files_with_extension(directory_path, extension)
dataset_path = f'{directory_path}/episode_{episode_idx}.hdf5'
video_path=f'/home/philaptop/wfy/dp_data/{episode_idx}'
data_dict = {
        '/observations/qpos': [],
        '/action': [],
        '/eef_qpos': [],
        '/observations/images/gripper' : [],
        '/observations/images/top' : [],
        # '/observations/velocity': [],
        # '/observations/current': [],
        }


class SampleNode(Node):
    def __init__(self):
        super().__init__('SampleNode')
        self.master_publisher_ = self.create_publisher(RobotCmd, "/arm_master_cmd", 10)
        self.follow_publisher_ = self.create_publisher(RobotCmd, "/arm_follow_cmd", 10)
        self.subscription = self.create_subscription(
            RobotStatus, '/arm_master_status', self.follow_callback, 10)
        self.subscription  # prevent unused variable warning

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.follow_cmd = RobotCmd()

        self.start_follow = False
        
        self.keyboard_listener = keyboard.Listener(on_press=self.on_press)
        self.keyboard_listener.start()

    def timer_callback(self):
        self.follow_publisher_.publish(self.follow_cmd)
        if self.start_follow:
            master_cmd = RobotCmd()
            master_cmd.mode = 3
            self.master_publisher_.publish(master_cmd)
    
    def master_publish(self,cmd: RobotCmd):
        self.master_publisher_.publish(cmd)

    def follow_publish(self,cmd: RobotCmd):
        self.follow_publisher_.publish(cmd)


    def follow_callback(self, msg):
        if self.start_follow:
            self.follow_cmd.mode = 5
            self.follow_cmd.joint_pos = msg.joint_pos[:6]
            self.follow_cmd.end_pos = msg.end_pos
            self.follow_cmd.gripper = msg.joint_pos[6] * 5
            # self.follow_cmd.joint_vel = msg.joint_vel
            # self.follow_cmd.joint_cur = msg.joint_cur
    
    def on_press(self,key):
        global finish_recording
        if key == keyboard.Key.esc:
            self.start_follow = False
            finish_recording = True
            rclpy.logging.get_logger("sample_node").info("Arm Reset to initial Condition")
            cmd = RobotCmd()
            cmd.mode = 1        # Reset to initial Condition
            
            self.follow_publisher_.publish(cmd)
            self.follow_cmd.mode = 1
            self.master_publisher_.publish(cmd)
            rclpy.logging.get_logger("sample_node").info("Arm Reset to initial Condition")
            # time.sleep(1)
            # rclpy.shutdown()


def callback(img1, img2, follow_status):
    global data_dict, step, Max_step, dataset_path, video_path, finish_recording
    
    save=True
    bridge = CvBridge()
    image_gripper = bridge.imgmsg_to_cv2(img1, "bgr8")
    image_top = bridge.imgmsg_to_cv2(img2, "bgr8")
    eef_qpos = np.array(follow_status.end_pos)
    action = np.array(follow_status.joint_pos)
    qpos =np.array(follow_status.joint_pos)
    # velocity = np.array(follow_status.joint_vel)
    # current = np.array(follow_status.joint_cur)

    #print("eef_qpos:", eef_qpos)
    #print("action:", action)
    # if save:
    if not finish_recording:
        data_dict["/eef_qpos"].append(eef_qpos)
        data_dict["/action"].append(action)
        data_dict["/observations/qpos"].append(qpos)
        data_dict["/observations/images/gripper"].append(image_gripper)
        data_dict["/observations/images/top"].append(image_top)
        # data_dict["/observations/velocity"].append(velocity)
        # data_dict["/observations/current"].append(current)

        step += 1

    # canvas = np.zeros((480, 1280, 3), dtype=np.uint8)

    # # Draw the image on canvas
    # # canvas[:, :640, :] = image_left
    # # canvas[:, 640:1280, :] = image_mid
    # # canvas[:, 1280:, :] = image_right
    # canvas[:, :640, :] = image_gripper
    # canvas[:, 640:, :] = image_top

    # # Show the things in a window
    # cv2.imshow('Multi Camera Viewer', canvas)
    # cv2.waitKey(1)

    # print(finish_recording)
    if step >= Max_step or finish_recording:
        print('end__________________________________')
        with h5py.File(dataset_path,'w',rdcc_nbytes=1024 ** 2 * 10) as root:
            root.attrs['sim'] = True
            obs = root.create_group('observations')
            image = obs.create_group('images')

            # _ = image.create_dataset('gripper', (Max_step, 480, 640, 3), dtype='uint8',
            #                         chunks=(1, 480, 640, 3), )
            # _ = image.create_dataset('top', (Max_step, 480, 640, 3), dtype='uint8',
            #                         chunks=(1, 480, 640, 3), )
            # _ = obs.create_dataset('qpos',(Max_step,7))
            # _ = root.create_dataset('action',(Max_step,7))
            # _ = root.create_dataset('eef_qpos',(Max_step,6))
            # # _ = root.create_dataset('velocity',(Max_step,7))
            # # _ = root.create_dataset('current',(Max_step,7))

            #---------------------- Mine ------------------
            _ = image.create_dataset('gripper', (step, 480, 640, 3), dtype='uint8',
                                    chunks=(1, 480, 640, 3), )
            _ = image.create_dataset('top', (step, 480, 640, 3), dtype='uint8',
                                    chunks=(1, 480, 640, 3), )
            _ = obs.create_dataset('qpos',(step,7))
            _ = root.create_dataset('action',(step,7))
            _ = root.create_dataset('eef_qpos',(step,6))
            # _ = root.create_dataset('velocity',(Max_step,7))
            # _ = root.create_dataset('current',(Max_step,7))

            #---------------------- Mine ------------------


            for name, array in data_dict.items():
                root[name][...] = array
            gipper_images = root['/observations/images/gripper'][...]
            top_images = root['/observations/images/top'][...]
            images = np.concatenate([gipper_images,top_images],axis=2)

            video_path = f'{video_path}video.mp4'  # Assuming dataset_path ends with ".hdf5"
            height, width, _ = images[0].shape
            fps = 15  # Publish Rate 10 Hz
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            video_writer = cv2.VideoWriter(video_path, fourcc, fps, (width, height))
            for img in images:
                video_writer.write(img)
            video_writer.release()
        
        rclpy.logging.get_logger("sample_node").info("Sampling Finished!")
        rclpy.shutdown()

        
def main():
    #config my camera
    time.sleep(1)  # wait 2s to start
    rclpy.init()
    
    node = SampleNode()
    
    master_cmd = RobotCmd()
    node.follow_cmd.mode = 1                # Reset to initial Condition Follow Arm
    master_cmd.mode = 1                     # Reset to initial Condition Master Arm
    node.master_publish(master_cmd)         # Publish to Master Arm
    time.sleep(2)

    master_cmd.mode = 3                     # Master Arm Set to gravity compenstaion mode, free to move
    node.master_publish(master_cmd)         # Publish to Master Arm
    node.follow_cmd.mode = 5                # Follow Arm Set to End Effector Control Mode

    node.get_logger().info("Arm Set to master-follow mode!")
    node.get_logger().info("Press 'ESC' to stop recording and reset the arm to initial condition.")
    node.start_follow = True

    # master1_pos = Subscriber("master1_pos_back",PosCmd)
    # master2_pos = Subscriber("master2_pos_back",PosCmd)
    # follow1_pos = Subscriber("follow1_pos_back",PosCmd)
    # follow2_pos = Subscriber("follow2_pos_back",PosCmd)
    # # master1 = Subscriber("joint_control",JointControl)
    # master2 = Subscriber("joint_control2",JointControl)
    # # follow1 = Subscriber("joint_information",JointInformation)
    # follow2 = Subscriber("joint_information2",JointInformation)
    img1 = Subscriber(node, Image,"/camera1/camera/color/image_rect_raw")
    img2 = Subscriber(node, Image,"/camera2/camera/color/image_rect_raw")
    # master_status = Subscriber(node, RobotStatus,"/arm_master_status")
    follow_status = Subscriber(node, RobotStatus,"/arm_master_status")
    # follow_status = node.subscription


    # image_left = Subscriber("left_camera",Image)
    # image_right = Subscriber("right_camera",Image)
    # ats = ApproximateTimeSynchronizer([master2,follow2,follow2_pos,image_mid,image_right],slop=0.03,queue_size=2)
    ats = ApproximateTimeSynchronizer([img1, img2, follow_status],slop=0.03,queue_size=2)
    ats.registerCallback(callback)

    # signal.signal(signal.SIGINT, node.sigint_hanler)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        pass

if __name__ =="__main__":
    main()
