"""
Usage:
(robodiff)$ python eval_real_robot.py -i <ckpt_path> -o <save_dir> --robot_ip <ip_of_ur5>

================ Human in control ==============
Robot movement:
Move your SpaceMouse to move the robot EEF (locked in xy plane).
Press SpaceMouse right button to unlock z axis.
Press SpaceMouse left button to enable rotation axes.

Recording control:
Click the opencv window (make sure it's in focus).
Press "C" to start evaluation (hand control over to policy).
Press "Q" to exit program.

================ Policy in control ==============
Make sure you can hit the robot hardware emergency-stop button quickly! 

Recording control:
Press "S" to stop evaluation and gain control back.
"""

import os
import sys

# sys.path.append("/home/dc/mambaforge/envs/robodiff/lib/python3.9/site-packages")
# sys.path.append("/home/philaptop/mambaforge/envs/robodiff/lib/python3.9/site-packages")
# sys.path.append(
#     "/media/dc/CLEAR/arx/dp_inference/dp"
# )

import time
import math
from multiprocessing.managers import SharedMemoryManager
import click
import cv2
import numpy as np
import torch
import dill
import hydra
import pathlib
import skvideo.io
from omegaconf import OmegaConf
import scipy.spatial.transform as st


from diffusion_policy.common.precise_sleep import precise_wait
from diffusion_policy.real_world.real_inference_util import get_real_obs_resolution, get_real_obs_dict
from diffusion_policy.common.pytorch_util import dict_apply
from diffusion_policy.workspace.base_workspace import BaseWorkspace
from diffusion_policy.policy.base_image_policy import BaseImagePolicy
from diffusion_policy.common.cv2_util import get_image_transform

from diffusion_policy.shared_memory.shared_memory_ring_buffer import SharedMemoryRingBuffer

import rclpy
import rclpy.logging
from rclpy.node import Node
from message_filters import ApproximateTimeSynchronizer, Subscriber

from arm_control.msg import PosCmd
from arx5_arm_msg.msg import RobotCmd, RobotStatus
from sensor_msgs.msg import Image
import threading
from cv_bridge import CvBridge
import matplotlib.pyplot as plt

gripper_width_min = -0.016
gripper_width_max = 0.4
actual_gripper_width_min = 0.0
actual_gripper_width_max = 5.0

OmegaConf.register_new_resolver("eval", eval, replace=True)
np.set_printoptions(suppress=True)


class ArxControl(Node):

    def __init__(self):
        super().__init__('arx_control')

        self.pub_ = self.create_publisher(RobotCmd, '/arm_follow_cmd', 10)
        # self.sub_ = self.create_subscription(
        #         RobotStatus,'arm_status', self.listener_callback,10)
        self.pub_
        # self.sub_  # prevent unused variable warning
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cmd = RobotCmd()
        self.cmd.mode = 5

        loop_rate = 0.1
        self.rate = self.create_rate(loop_rate, self.get_clock())

    # def listener_callback(self, msg):
    #     self.get_logger().info('I heard: "%s"' % msg)
    #     self.arm_status_ = msg
    
    def timer_callback(self):
        # msg.data[0] = 0.0
        self.pub_.publish(self.cmd)
        # self.get_logger().info('Publishing: "%s"' % msg)

    def set_cmd(self, cmd):
        self.cmd_ = cmd

    def get_arm_status(self):
        return self.arm_status_


# @click.command()
# @click.option("--input_path", "-ip", required=True, help="Path to checkpoint",)
# @click.option("--output_path", "-op", required=True, default="/home/dc/Desktop/diffusion-policy-inference-master/data/", help="Output video path")
# @click.option("--frequency", "-f", default=10, type=int, help="control frequency")
# @click.option("--steps_per_inference", "-si", default=8, type=int, help="Action horizon for inference.")
# @click.option("--max_step", "-mt", default=300, type=int, help="Max time step for deployment in an episode")

# @profile
def main(
    # input_path,
    # output_path,
    # frequency,
    # steps_per_inference,
    # max_step
):
    steps_per_inference = 8
    output_path = "/home/philaptop/wfy/output"
    max_step = 1000

    rclpy.init()
    node = ArxControl()

    global obs_ring_buffer, current_step
    frequency = 10
    # print(os.getcwd())
    ckpt_path = "/home/philaptop/wfy/ckpt/14.15.51_train_diffusion_unet_image_drawer/checkpoints/latest.ckpt"

    dt = 1 / frequency
    video_capture_fps = 30
    max_obs_buffer_size = 30

    # load checkpoint
    # ckpt_path = input_path
    
    payload = torch.load(open(ckpt_path, "rb"), map_location="cpu", pickle_module=dill)
    cfg = payload["cfg"]
    # print(cfg)
    # exit()
    # cfg._target_ = "diffusion_policy." + cfg._target_  # can delete
    # cfg._target_ = "diffusion_policy.workspace.train_workspace.TrainDiffusionWorkspace"
    # print(sys.path)
    import wandb
    import diffusion_policy.workspace.train_diffusion_unet_image_workspace
    cfg._target_ = "diffusion_policy.workspace.train_diffusion_unet_image_workspace.TrainDiffusionUnetImageWorkspace"

    cls = hydra.utils.get_class(cfg._target_)
    workspace: BaseWorkspace = cls(cfg)
    workspace.load_payload(payload, exclude_keys=None, include_keys=None)

    policy: BaseImagePolicy
    policy = workspace.model
    if cfg.training.use_ema:
        policy = workspace.ema_model

    device = torch.device("cuda:0")
    policy.eval().to(device)

    ## set inference params
    policy.num_inference_steps = 16  # DDIM inference iterations
    policy.n_action_steps = policy.horizon - policy.n_obs_steps + 1

    obs_res = get_real_obs_resolution(cfg.task.shape_meta)
    n_obs_steps = cfg.n_obs_steps
    print("n_obs_steps: ", n_obs_steps)
    print("steps_per_inference:", steps_per_inference)
    
    # shared memory
    shm_manager = SharedMemoryManager()
    shm_manager.start()

    examples = dict()
    examples["gripper"] = np.empty(shape=obs_res[::-1] + (3,), dtype=np.uint8)
    examples["top"] = np.empty(shape=obs_res[::-1] + (3,), dtype=np.uint8)
    examples["eef_qpos"] = np.empty(shape=(6,), dtype=np.float64)
    examples["qpos"] = np.empty(shape=(7,), dtype=np.float64)
    # examples["mid_orig"] = np.empty(shape=(480, 640, 3, ), dtype=np.uint8)
    # examples["right_orig"] = np.empty(shape=(480, 640, 3, ), dtype=np.uint8)
    examples["timestamp"] = 0.0
    obs_ring_buffer = SharedMemoryRingBuffer.create_from_examples(
        shm_manager=shm_manager,
        examples=examples,
        get_max_k=max_obs_buffer_size,
        get_time_budget=0.2,
        put_desired_frequency=video_capture_fps,
    )

    # visuilization

    # fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    # output_video = cv2.VideoWriter(output_path + "/replay.mp4", fourcc, 1.0, (448, 224))

    session_dir = os.listdir(output_path)
    idx_max = 0
    for session in session_dir:
        idx_str = session.split('_')[-1]
        idx_max = max(idx_max, int(idx_str))
    new_dir_str = output_path + "/episode_" + str(idx_max + 1)
    new_dir = pathlib.Path(new_dir_str)
    new_dir.mkdir()

    mid_img_dir = new_dir.joinpath("mid")
    right_img_dir = new_dir.joinpath("right")
    mid_img_dir.mkdir()
    right_img_dir.mkdir()

    fourcc_visualization = cv2.VideoWriter_fourcc(*'mp4v')
    output_video_visualization = cv2.VideoWriter(new_dir_str + "/video.mp4", fourcc_visualization, 30.0, (1280, 480))

    fourcc_mid = cv2.VideoWriter_fourcc(*'mp4v')
    output_video_mid = cv2.VideoWriter(new_dir_str + "/mid/mid.mp4", fourcc_mid, 30.0, (640, 480))

    fourcc_right = cv2.VideoWriter_fourcc(*'mp4v')
    output_video_right = cv2.VideoWriter(new_dir_str + "/right/right.mp4", fourcc_right, 30.0, (640, 480))
    
    current_step = 0
    start_time = math.inf

    # ros init and subscriber
    # rclpy.init("eval_real_ros")
    arm_status = Subscriber(node, RobotStatus, "/arm_follow_status")
    # qpos = Subscriber("joint_information2", JointInformation)

    img1 = Subscriber(node, Image,"/camera1/camera/color/image_rect_raw")
    img2 = Subscriber(node, Image,"/camera2/camera/color/image_rect_raw")
    # control_robot2 = rospy.Publisher("test_right", JointControl, queue_size=10)
    # control_robot2 = rclpy.Publisher("follow_joint_control_2", JointControl, queue_size=10)
    # arm_control_pub = rclpy.create_publisher(PosCmd, 'arm_cmd', 10)
    ats = ApproximateTimeSynchronizer(
        [arm_status, img1, img2], queue_size=10, slop=0.1
    )
    ats.registerCallback(
        lambda *msg: callback(*msg, output_video_visualization, output_video_mid, output_video_right, max_step, start_time)
    )
    # ats.registerCallback(callback)
    # rate = rospy.Rate(frequency)


    # # data ??
    last_data = None
    # right_control = JointControl()
    # joint_data = node.arm_status_
    

    # start
    # ========== policy control loop ==============
    # start episode
    policy.reset()
    start_delay = 1.0
    eval_t_start = time.time() + start_delay
    t_start = time.monotonic() + start_delay
    frame_latency = 1/30
    precise_wait(eval_t_start - frame_latency, time_func=time.time)
    print("Started!")
    iter_idx = 0
    start_time = time.time()
    

    # inference loop
    while rclpy.ok():
        for i in range(400):
            rclpy.spin_once(node, timeout_sec = 1)
        # rclpy.spin(node)
        # node.rate.sleep()
        
        if current_step >= max_step:
            break

        t_cycle_end = t_start + (iter_idx + steps_per_inference) * dt

        # get observation
        k = math.ceil(n_obs_steps * (video_capture_fps / frequency))
        last_data = obs_ring_buffer.get_last_k(k=k, out=last_data)
        last_timestamp = last_data["timestamp"][-1]
        obs_align_timestamps = last_timestamp - (np.arange(n_obs_steps)[::-1] * dt)

        obs_dict = dict()
        this_timestamps = last_data["timestamp"]
        this_idxs = list()
        for t in obs_align_timestamps:
            is_before_idxs = np.nonzero(this_timestamps <= t)[0]
            this_idx = 0
            if len(is_before_idxs) > 0:
                this_idx = is_before_idxs[-1]
            this_idxs.append(this_idx)
        for key in last_data.keys():
            obs_dict[key] = last_data[key][this_idxs]

        obs_timestamps = obs_dict["timestamp"]
        print("Got Observation!")

        # run inference
        with torch.no_grad():
            test_t_start = time.perf_counter()
            obs_dict_np = get_real_obs_dict(
                env_obs=obs_dict, shape_meta=cfg.task.shape_meta)
            obs_dict = dict_apply(obs_dict_np,
                lambda x: torch.from_numpy(x).unsqueeze(0).to(torch.device("cuda:0")))

            # image = obs_dict['right'][0, 1, :, :, :]
            # image_converted = np.transpose(image.cpu().numpy(), (1, 2, 0))
            # plt.imshow(image_converted)
            # plt.show()
            # import ipdb;ipdb.set_trace()
            # exit()
            print(obs_dict['qpos'])
            result = policy.predict_action(obs_dict)
            
            action = result["action"][0].detach().to("cpu").numpy()
            # print(action)
            # print("----------------------------")
            print(f"Inference latency {steps_per_inference/(time.perf_counter() - test_t_start)}")

        # visualization
        mid_img = obs_dict["gripper"][0, -1]
        mid_img = mid_img.permute(1, 2, 0)
        mid_img = mid_img.cpu().numpy()
        mid_img = (255 * mid_img).astype(np.uint8)

        right_img = obs_dict["top"][0, -1]
        right_img = right_img.permute(1, 2, 0)
        right_img = right_img.cpu().numpy()
        right_img = (255 * right_img).astype(np.uint8)

        # vis_img = np.zeros((224, 448, 3), dtype=np.uint8)
        # vis_img[:, :224, :] = mid_img
        # vis_img[:, 224:448, :] = right_img

        vis_img = np.zeros((480, 1280, 3), dtype=np.uint8)
        vis_img[:, :640, :] = mid_img
        vis_img[:, 640:1280, :] = right_img

        # cv2.imshow('Multi Camera Viewer', vis_img)
        # output_video.write(vis_img)
        key_stroke = cv2.pollKey()
        if key_stroke == ord('q'):
            # Stop episode
            # Hand control back to human
            print('Stopped.')
            break

        # preprocess action
        action = action[:steps_per_inference, :]
        action_timestamps = (np.arange(len(action), dtype=np.float64)) * dt + obs_timestamps[-1]

        action_exec_latency = 0.01
        curr_time = time.time()
        is_new = action_timestamps > (curr_time + action_exec_latency)

        if np.sum(is_new) == 0:
            action = action[[-1]]
            next_step_idx = int(np.ceil((curr_time - eval_t_start) / dt))
            action_timestamp = eval_t_start + (next_step_idx) * dt
            print("Over budget", action_timestamp - curr_time)
            action_timestamps = np.array([action_timestamp])
        else:
            action = action[is_new]
            action_timestamps = action_timestamps[is_new]
            
        # execute actions
        print("Execute Action!")
        # print(action)
        print("==================================")
        for item in action:
            # right_control.joint_pos = item

            # control_robot2.publish(right_control)

            # rate.sleep()

            # node.cmd.joint_pos = np.array(item[:6], dtype=np.float64)
            # node.cmd.gripper = float(item[6])

            # node.rate.sleep()
            
            # rclpy.spin_once(node,timeout_sec=0.1)

            time.sleep(2)
            print(item)
            # precise_wait(0.1)

        precise_wait(t_cycle_end - frame_latency)
        iter_idx += steps_per_inference

    # output_video.release()
    output_video_visualization.release()
    output_video_mid.release()
    output_video_right.release()
    cv2.destroyAllWindows()

    

def callback(arm_status, image_mid, image_right, output_video_visualization, output_video_mid, output_video_right, max_step, start_time):
    global obs_ring_buffer, current_step
    # rclpy.logging.get_logger("eval_real_ros").info("Received data")

    mid = image_mid
    right = image_right
    bridge = CvBridge()
    receive_time = time.time()

    obs_data = dict()
    eef_qpos = arm_status.end_pos
    qpos = arm_status.joint_pos
    # obs_data["eef_qpos"] = np.array(
    #     [
    #         eef_qpos.x,
    #         eef_qpos.y,
    #         eef_qpos.z,
    #         eef_qpos.roll,
    #         eef_qpos.pitch,
    #         eef_qpos.yaw,
    #         eef_qpos.gripper,
    #     ]
    # )

    obs_data["eef_qpos"] = eef_qpos
    gripper_width = qpos[6]

    # gripper width calibration
    # gripper_width = (gripper_width - actual_gripper_width_min) / (actual_gripper_width_max - actual_gripper_width_min) * \
    #                 (gripper_width_max - gripper_width_min) + gripper_width_min

    # gripper_width = gripper_width / 12 - 0.016 
    # obs_data["qpos"] = np.array(
    #     [
    #         qpos.joint_pos[0],
    #         qpos.joint_pos[1],
    #         qpos.joint_pos[2],
    #         qpos.joint_pos[3],
    #         qpos.joint_pos[4],
    #         qpos.joint_pos[5],
    #         gripper_width
    #     ]
    # )
    obs_data["qpos"] = qpos

    # process images observation
    img1 = bridge.imgmsg_to_cv2(mid, "bgr8")
    img2 = bridge.imgmsg_to_cv2(right, "bgr8")
    # obs_data["gripper"] = transform(mid)
    # obs_data["top"] = transform(right)
    obs_data["gripper"] = img1
    obs_data["top"] = img2
    obs_data["timestamp"] = receive_time
    # obs_data["mid_orig"] = mid
    # obs_data["right_orig"] = right

    # img_to_save = np.zeros((480, 640, 3), dtype=np.uint8)
    # img_to_save[:,:,:] = mid
    # idx_max = 0
    # for saved_img in list(mid_img_dir.glob('*.jpg')):
    #     # print(saved_img.stem)
    #     idx_str = str(saved_img.stem).split('_')[-1]
    #     idx_max = max(idx_max, int(idx_str))
    # save_path = mid_img_dir.joinpath("mid_{}.jpg".format(idx_max + 1))
    # cv2.imwrite(str(save_path), img_to_save)

    # img_to_save = np.zeros((480, 640, 3), dtype=np.uint8)
    # img_to_save[:,:,:] = right
    # idx_max = 0
    # for saved_img in list(right_img_dir.glob('*.jpg')):
    #     # print(saved_img.stem)
    #     idx_str = str(saved_img.stem).split('_')[-1]
    #     idx_max = max(idx_max, int(idx_str))
    # save_path = right_img_dir.joinpath("right_{}.jpg".format(idx_max + 1))
    # cv2.imwrite(str(save_path), img_to_save)

    # save video
    if current_step < max_step and time.time() > start_time:
        output_video_mid.write(img1)
        output_video_right.write(img2)

        canvas = np.zeros((480, 1280, 3), dtype=np.uint8)
        canvas[:, :640, :] = img1
        canvas[:, 640:1280, :] = img2

        output_video_visualization.write(canvas)

        current_step += 1
    
    # cv2.imshow('Multi Camera Viewer2', canvas)
    # key_stroke = cv2.pollKey()
    # if cv2.waitKey(1) == ord("q"):
    #     rospy.signal_shutdown("User requested shutdown")
    
    # put data
    put_data = obs_data
    obs_ring_buffer.put(put_data, wait=False)

def transform(data, video_capture_resolution=(640, 480), obs_image_resolution=(224, 224)):
    color_tf = get_image_transform(
                input_res=video_capture_resolution,
                output_res=obs_image_resolution,
                # obs output rgb
                bgr_to_rgb=False,
            )
    data = np.array(data)
    tf_data = color_tf(data)
    return tf_data

if __name__ == "__main__":
    main()