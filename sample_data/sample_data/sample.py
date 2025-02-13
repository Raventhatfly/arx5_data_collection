#!/home/dc/anaconda3/envs/dc/bin/python
import time
import rclpy
import sys
import os
from message_filters import ApproximateTimeSynchronizer,Subscriber
# pypkg_path = os.path.join(os.path.expanduser('~'),"anaconda3/envs/")
# sys.path.append("/home/dc/anaconda3/envs/dc/lib/python3.8/site-packages")
import numpy as np
import cv2
import h5py
from cv_bridge import CvBridge
# from arm_control.msg import JointInformation
# from arm_control.msg import JointControl
# from arm_control.msg import PosCmd
import rclpy.publisher
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

global data_dict, step, Max_step, dataset_path 

# parameters
step = 0
Max_step = 200
directory_path = f'/home/dc/Downloads'
extension = '.hdf5' 
episode_idx = count_files_with_extension(directory_path, extension)
dataset_path = f'{directory_path}/episode_{episode_idx}.hdf5'
video_path=f'/home/dc/Downloads/{episode_idx}'
data_dict = {
        '/observations/qpos': [],
        '/action': [],
        '/eef_qpos': [],
        '/observations/images/gripper' : [],
        '/observations/images/top' : [],
        }

class SampleNode(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.master_publisher_ = self.create_publisher(RobotCmd, "/arm_control_master", 10)
        self.follow_publisher_ = self.create_publisher(RobotCmd, "/arm_control_master", 10)
        self.subscription = self.create_subscription(
            RobotStatus, 'arm_status', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.follow_cmd = RobotCmd()

        self.start_follow = False

    def timer_callback(self):
        self.follow_publisher_.publish(self.follow_cmd)
    
    def master_publish(self,cmd: RobotCmd):
        self.master_publisher_.publish(cmd)

    def listener_callback(self, msg):
        if self.start_follow:
            self.follow_cmd.mode = 4
            self.follow_cmd.joint_pos = msg.joint_pos
            self.follow_cmd.eef_pos = msg.eef_pos
            self.follow_cmd.gripper = msg.gripper


def callback(img1, img2, follow_status):
    global data_dict, step, Max_step, dataset_path,video_path
    
    save=True
    bridge = CvBridge()
    image_gripper = bridge.imgmsg_to_cv2(img1, "bgr8")
    image_top = bridge.imgmsg_to_cv2(img2, "bgr8")
    eef_qpos = np.array(follow_status.eef_pos)
    action = np.array(follow_status.joint_pos)
    qpos =np.array(follow_status.joint_pos)
    current = np.array(follow_status.joint_cur)

    #print("eef_qpos:", eef_qpos)
    #print("action:", action)
    if save:
        data_dict["/eef_qpos"].append(eef_qpos)
        data_dict["/action"].append(action)
        data_dict["/observations/qpos"].append(qpos)
        data_dict["/observations/images/mid"].append(image_gripper)
        data_dict["/observations/images/right"].append(image_top)

    canvas = np.zeros((480, 1280, 3), dtype=np.uint8)

    # Draw the image on canvas
    # canvas[:, :640, :] = image_left
    # canvas[:, 640:1280, :] = image_mid
    # canvas[:, 1280:, :] = image_right
    canvas[:, :640, :] = image_gripper
    canvas[:, 640:, :] = image_top

    # Show the things in a window
    cv2.imshow('Multi Camera Viewer', canvas)
    cv2.waitKey(1)

    step = step+1
    print(step)
    if step >= Max_step and save:
        print('end__________________________________')
        with h5py.File(dataset_path,'w',rdcc_nbytes=1024 ** 2 * 10) as root:
            root.attrs['sim'] = True
            obs = root.create_group('observations')
            image = obs.create_group('images')
            _ = image.create_dataset('mid', (Max_step, 480, 640, 3), dtype='uint8',
                                    chunks=(1, 480, 640, 3), )
            _ = image.create_dataset('right', (Max_step, 480, 640, 3), dtype='uint8',
                                    chunks=(1, 480, 640, 3), )
            _ = obs.create_dataset('qpos',(Max_step,7))
            _ = root.create_dataset('action',(Max_step,7))
            _ = root.create_dataset('eef_qpos',(Max_step,7))
            _ = root.create_dataset('current',(Max_step,7))

            for name, array in data_dict.items():
                root[name][...] = array
            gipper_images = root['/observations/images/gripper'][...]
            top_images = root['/observations/images/top'][...]
            images = np.concatenate([gipper_images,top_images],axis=2)

            video_path = f'{video_path}video.mp4'  # Assuming dataset_path ends with ".hdf5"
            height, width, _ = images[0].shape
            fps = 10  # Publish Rate 10 Hz
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            video_writer = cv2.VideoWriter(video_path, fourcc, fps, (width, height))
            for img in images:
                video_writer.write(img)
            video_writer.release()

        rclpy.shutdown()
        quit("sample successfully!")
        
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
    node.follow_cmd.mode = 4                # Follow Arm Set to End Effector Control Mode

    node.get_logger().info("Arm Set to master-follow mode!")

    # master1_pos = Subscriber("master1_pos_back",PosCmd)
    # master2_pos = Subscriber("master2_pos_back",PosCmd)
    # follow1_pos = Subscriber("follow1_pos_back",PosCmd)
    # follow2_pos = Subscriber("follow2_pos_back",PosCmd)
    # # master1 = Subscriber("joint_control",JointControl)
    # master2 = Subscriber("joint_control2",JointControl)
    # # follow1 = Subscriber("joint_information",JointInformation)
    # follow2 = Subscriber("joint_information2",JointInformation)
    img1 = Subscriber(node, Image,"/camera/camera/color/image_rect_raw")
    img2 = Subscriber(node, Image,"/camera/camera/color/image_rect_raw")
    master_status = Subscriber(node, PosCmd,"/arm_status")
    follow_status = Subscriber(node, PosCmd,"/arm_status")


    # image_left = Subscriber("left_camera",Image)
    # image_right = Subscriber("right_camera",Image)
    # ats = ApproximateTimeSynchronizer([master2,follow2,follow2_pos,image_mid,image_right],slop=0.03,queue_size=2)
    ats = ApproximateTimeSynchronizer([img1, img2, master_status, follow_status],slop=0.03,queue_size=2)
    ats.registerCallback(callback)
    
    rclpy.spin(node)

if __name__ =="__main__":
    main()
