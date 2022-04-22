#!/usr/bin/env python
# coding: utf-8

import csv
from enum import IntEnum
import time
import jsonrpclib
import subprocess
from subprocess import PIPE, Popen
from threading  import Thread
import sys
import logging
import re
from collections import OrderedDict
import threading
import queue

from gym import Env, error, spaces, utils
from stable_baselines3 import DQN, PPO, A2C, TD3, SAC
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.callbacks import BaseCallback, CallbackList, CheckpointCallback, EvalCallback
from stable_baselines3.common import results_plotter
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.results_plotter import load_results, ts2xy, plot_results
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common import results_plotter
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.results_plotter import load_results, ts2xy, plot_results

import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import multivariate_normal

import os
import requests
import shutil
import tempfile
import xml.etree.ElementTree as ET
from io import StringIO, BytesIO

import cv2
import dbr
import numpy as np
import torch
from PIL import Image
from IPython.display import clear_output
import gym
from cv2 import QRCodeDetector
from pyzbar import pyzbar

import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, moveBy, PCMD, moveTo
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged, PositionChanged, GpsLocationChanged, moveToChanged
from olympe.enums.ardrone3.PilotingState import FlyingStateChanged_State as FlyingState
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged, HomeChanged
from olympe.messages.common.CommonState import BatteryStateChanged
from olympe.messages.gimbal import set_target, attitude
from olympe.messages.camera import (
    set_camera_mode,
    set_photo_mode,
    take_photo,
    photo_progress,
)
from olympe.media import (
    media_created,
    resource_created,
    media_removed,
    resource_removed,
    resource_downloaded,
    indexing_state,
    delete_media,
    download_media,
    download_media_thumbnail,
    MediaEvent,
)

from collections import defaultdict

olympe.log.update_config({
    "loggers": {
        "olympe": {
                "handlers": []
            }
        },
        "ulog": {
            "level": "OFF",
            "handlers": [],
        }
})

#==========MANIPULATED VARIABLES==========#
DETECTION_DELAY = 0.5 # seconds
MAX_TIMESTEP = 15
LOG_DIR = "logs/"
MODEL_PATH = "models/fixed"
# DRONE_IP = os.environ.get("DRONE_IP", "10.202.0.1")
DRONE_IP = os.environ.get("DRONE_IP", "192.168.42.1")
LICENSE_KEY = "DLS2eyJvcmdhbml6YXRpb25JRCI6IjIwMDAwMSJ9"
#=========================================#

DRONE_MEDIA_PORT = os.environ.get("DRONE_MEDIA_PORT", "80")
DRONE_RTSP_PORT = os.environ.get("DRONE_RTSP_PORT")

ANAFI_URL = "http://{}/".format(DRONE_IP)
ANAFI_MEDIA_API_URL = ANAFI_URL + "api/v1/media/medias/"

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

formatter = logging.Formatter('%(created)f,%(message)s')

os.makedirs(LOG_DIR, exist_ok=True)

timestr = time.strftime("%Y%m%d-%H%M%S")
filename = timestr + ".log"
abs_filename = os.path.join(LOG_DIR, filename)
file_handler = logging.FileHandler(abs_filename, mode='w')

file_handler.setLevel(logging.INFO)
file_handler.setFormatter(formatter)

logger.addHandler(file_handler)

class Action:
    def __init__(self, drone):
        self.drone = drone
        self.home = self.drone.get_state(GpsLocationChanged)
        
        self.current_cell = self._get_cell(13)
        self.invalid_left_cells = [1, 6, 11, 16, 21]
        self.invalid_forward_cells = [1, 2, 3, 4, 5]
        self.invalid_right_cells = [5, 10, 15, 20, 25]
        self.invalid_backward_cells = [21, 22, 23, 24, 25]
        
        self.Move = IntEnum(
            'MOVE',
            'FORWARD BACKWARD LEFT RIGHT FORWARD_LEFT FORWARD_RIGHT BACKWARD_LEFT BACKWARD_RIGHT HOVER',
            start=0
        )
        
    def take_action(self, action):
        next_cell_id = self._get_next_cell_id(action)
        next_cell = self._get_cell(next_cell_id)
        
        old_cell_id, new_cell_id = self.current_cell["id"], next_cell["id"]
        if old_cell_id == new_cell_id: 
            return old_cell_id, new_cell_id, self._get_action_name(action)
        
        self._move_to_cell(next_cell)
        
        self.current_cell = next_cell
        
        return old_cell_id, new_cell_id, self._get_action_name(action)
        
    def reset(self):
        next_cell = self._get_cell(13)
        self._move_to_cell(next_cell)
        
        old_cell_id, new_cell_id = self.current_cell["id"], next_cell["id"]
        self.current_cell = next_cell
        
        return old_cell_id, new_cell_id
    
    def _get_cell(self, cell_id):
        return self._cell_coords[cell_id - 1]
    
    def _get_action_name(self, action):
        direction = str(self.Move(action)).split(".")[1]
        code = ""
        for i in direction.split("_"):
            if i != "":
                code += i[0].upper()
        
        return code
    
    def _move_to_cell(self, next_cell):        
        self.drone(
            moveTo(next_cell["latitude"],  next_cell["longitude"], next_cell["altitude"], "HEADING_DURING", 90.0)
            >> moveToChanged(status="DONE", _timeout=15)
        ).wait()
    
    def _get_next_cell_id(self, action):
        if action == self.Move.HOVER:
            return self.current_cell["id"]
        elif action == self.Move.LEFT:
            if self.current_cell["id"] in self.invalid_left_cells:
                return self.current_cell["id"]
            next_cell_id = self.current_cell["id"] - 1
        elif action == self.Move.RIGHT:
            if self.current_cell["id"] in self.invalid_right_cells:
                return self.current_cell["id"]
            next_cell_id = self.current_cell["id"] + 1
        elif action == self.Move.FORWARD:
            if self.current_cell["id"] in self.invalid_forward_cells:
                return self.current_cell["id"]
            next_cell_id = self.current_cell["id"] - 5
        elif action == self.Move.BACKWARD:
            if self.current_cell["id"] in self.invalid_backward_cells:
                return self.current_cell["id"]
            next_cell_id = self.current_cell["id"] + 5
        elif action == self.Move.FORWARD_RIGHT:
            if self.current_cell["id"] in self.invalid_forward_cells + self.invalid_right_cells:
                return self.current_cell["id"]
            next_cell_id = self.current_cell["id"] - 4
        elif action == self.Move.FORWARD_LEFT:
            if self.current_cell["id"] in self.invalid_forward_cells + self.invalid_left_cells:
                return self.current_cell["id"]
            next_cell_id = self.current_cell["id"] - 6
        elif action == self.Move.BACKWARD_RIGHT:
            if self.current_cell["id"] in self.invalid_backward_cells + self.invalid_right_cells:
                return self.current_cell["id"]
            next_cell_id = self.current_cell["id"] + 6
        elif action == self.Move.BACKWARD_LEFT:
            if self.current_cell["id"] in self.invalid_backward_cells + self.invalid_left_cells:
                return self.current_cell["id"]
            next_cell_id = self.current_cell["id"] + 4
            
        return next_cell_id
    
    @property
    def _cell_coords(self):
        altitude = 2.5
        dlong = 6.8e-5 * (2/8.5) # in degrees == 5 meters along x-axis (forward[+]-backward[-])
        dlat = 7.2e-5 * (3.2/8.5) # in degrees == 8 meters along y-axis (left[+]-right[-])
        
        home_lat = self.home["latitude"]
        home_long = self.home["longitude"]
        
        return [
            # cell no. 1
            OrderedDict([('id', 1),
                         ('latitude', home_lat + 2 * dlat),
                         ('longitude', home_long + 2 * dlong),
                         ('altitude', altitude)]),
            # cell no. 2
            OrderedDict([('id', 2),
                         ('latitude', home_lat + 1 * dlat),
                         ('longitude', home_long + 2 * dlong),
                         ('altitude', altitude)]),
            # cell no. 3
            OrderedDict([('id', 3),
                         ('latitude', home_lat + 0 * dlat),
                         ('longitude', home_long + 2 * dlong),
                         ('altitude', altitude)]),
            # cell no. 4
            OrderedDict([('id', 4),
                         ('latitude', home_lat + -1 * dlat),
                         ('longitude', home_long + 2 * dlong),
                         ('altitude', altitude)]),
            # cell no. 5
            OrderedDict([('id', 5),
                         ('latitude', home_lat + -2 * dlat),
                         ('longitude', home_long + 2 * dlong),
                         ('altitude', altitude)]),
            # cell no. 6
            OrderedDict([('id', 6),
                         ('latitude', home_lat + 2 * dlat),
                         ('longitude', home_long + 1 * dlong),
                         ('altitude', altitude)]),
            # cell no. 7
            OrderedDict([('id', 7),
                         ('latitude', home_lat + 1 * dlat),
                         ('longitude', home_long + 1 * dlong),
                         ('altitude', altitude)]),
            # cell no. 8
            OrderedDict([('id', 8),
                         ('latitude', home_lat + 0 * dlat),
                         ('longitude', home_long + 1 * dlong),
                         ('altitude', altitude)]),
            # cell no. 9
            OrderedDict([('id', 9),
                         ('latitude', home_lat + -1 * dlat),
                         ('longitude', home_long + 1 * dlong),
                         ('altitude', altitude)]),
            # cell no. 10
            OrderedDict([('id', 10),
                         ('latitude', home_lat + -2 * dlat),
                         ('longitude', home_long + 1 * dlong),
                         ('altitude', altitude)]),
            # cell no. 11
            OrderedDict([('id', 11),
                         ('latitude', home_lat + 2 * dlat),
                         ('longitude', home_long + 0 * dlong),
                         ('altitude', altitude)]),
            # cell no. 12
            OrderedDict([('id', 12),
                         ('latitude', home_lat + 1 * dlat),
                         ('longitude', home_long + 0 * dlong),
                         ('altitude', altitude)]),
            # cell no. 13
            OrderedDict([('id', 13),
                         ('latitude', home_lat + 0 * dlat),
                         ('longitude', home_long + 0 * dlong),
                         ('altitude', altitude)]),
            # cell no. 14
            OrderedDict([('id', 14),
                         ('latitude', home_lat + -1 * dlat),
                         ('longitude', home_long + 0 * dlong),
                         ('altitude', altitude)]),
            # cell no. 15
            OrderedDict([('id', 15),
                         ('latitude', home_lat + -2 * dlat),
                         ('longitude', home_long + 0 * dlong),
                         ('altitude', altitude)]),
            # cell no. 16
            OrderedDict([('id', 16),
                         ('latitude', home_lat + 2 * dlat),
                         ('longitude', home_long + -1 * dlong),
                         ('altitude', altitude)]),
            # cell no. 17
            OrderedDict([('id', 17),
                         ('latitude', home_lat + 1 * dlat),
                         ('longitude', home_long + -1 * dlong),
                         ('altitude', altitude)]),
            # cell no. 18
            OrderedDict([('id', 18),
                         ('latitude', home_lat + 0 * dlat),
                         ('longitude', home_long + -1 * dlong),
                         ('altitude', altitude)]),
            # cell no. 19
            OrderedDict([('id', 19),
                         ('latitude', home_lat + -1 * dlat),
                         ('longitude', home_long + -1 * dlong),
                         ('altitude', altitude)]),
            # cell no. 20
            OrderedDict([('id', 20),
                         ('latitude', home_lat + -2 * dlat),
                         ('longitude', home_long + -1 * dlong),
                         ('altitude', altitude)]),
            # cell no. 21
            OrderedDict([('id', 21),
                         ('latitude', home_lat + 2 * dlat),
                         ('longitude', home_long + -2 * dlong),
                         ('altitude', altitude)]),
            # cell no. 22
            OrderedDict([('id', 22),
                         ('latitude', home_lat + 1 * dlat),
                         ('longitude', home_long + -2 * dlong),
                         ('altitude', altitude)]),
            # cell no. 23
            OrderedDict([('id', 23),
                         ('latitude', home_lat + 0 * dlat),
                         ('longitude', home_long + -2 * dlong),
                         ('altitude', altitude)]),
            # cell no. 24
            OrderedDict([('id', 24),
                         ('latitude', home_lat + -1 * dlat),
                         ('longitude', home_long + -2 * dlong),
                         ('altitude', altitude)]),
            # cell no. 25
            OrderedDict([('id', 25),
                         ('latitude', home_lat + -2 * dlat),
                         ('longitude', home_long + -2 * dlong),
                         ('altitude', altitude)]),
            ]
        
    def __len__(self):
        return len(self.Move)


class Streaming(threading.Thread):
    def decode(self, frame):

        before = time.time()
        results = self.reader.decode_buffer(frame)
        after = time.time()

        COLOR_RED = (0,0,255)
        thickness = 2
        margin = 1
        text_x = 10; text_y = 20
        if results != None:
            found = len(results)
            for result in results:
                # print("Format: %s, Text: %s" % (result.barcode_format_string, result.barcode_text))
                text = result.barcode_text 
                points = result.localization_result.localization_points
                data = np.array([[points[0][0], points[0][1]], [points[1][0], points[1][1]], [points[2][0], points[2][1]], [points[3][0], points[3][1]]])
                cv2.drawContours(image=frame, contours=[data], contourIdx=-1, color=COLOR_RED, thickness=thickness, lineType=cv2.LINE_AA)
                cv2.putText(frame, result.barcode_text, (np.min(data[:,0]) - margin, np.min(data[:,1]) - margin), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_RED)
                idx = int(result.barcode_text) - 1
                try:
                    self.detected_targets[idx] = True
                except (ValueError, IndexError):
                    pass

            # cv2.putText(frame, '%.2f s, barcode found: %d' % (after - before, found), (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_RED)
        # else:
            # cv2.putText(frame, '%.2f s, barcode found: %d' % (after - before, 0), (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_RED)

        return results
    
    def __init__(self, drone, num_targets):
        self.drone = drone
        self.frame_queue = queue.Queue()
        self.flush_queue_lock = threading.Lock()
        self.frame_num = 0 
        self.renderer = None
        
        self.reader = dbr.BarcodeReader()
        self.reader.init_license(LICENSE_KEY)
        
        name = os.path.join(LOG_DIR, timestr + "_cv.mp4")
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.out = cv2.VideoWriter(name, fourcc, 30.10, (1280,720))

        self.is_detecting = False
        self.num_targets = num_targets
        self.detected_targets = np.zeros(self.num_targets, dtype=bool)
        
        super().__init__()
        super().start()


    def start(self):
        self.drone.streaming.set_output_files(
            video=os.path.join(LOG_DIR, timestr + ".mp4"),
        )
        
        self.drone.streaming.set_callbacks(
            raw_cb=self.yuv_frame_cb,
            h264_cb=self.h264_frame_cb,
            start_cb=self.start_cb,
            end_cb=self.end_cb,
            flush_raw_cb=self.flush_cb,
        )

        self.drone.streaming.start()

    def stop(self):
        if self.renderer is not None:
            self.renderer.stop()

        self.drone.streaming.stop()

    def yuv_frame_cb(self, yuv_frame):
        """
        This function will be called by Olympe for each decoded YUV frame.
            :type yuv_frame: olympe.VideoFrame
        """
        yuv_frame.ref()
        self.frame_queue.put_nowait(yuv_frame)

    def flush_cb(self, stream):
        if stream["vdef_format"] != olympe.VDEF_I420:
            return True
        with self.flush_queue_lock:
            while not self.frame_queue.empty():
                self.frame_queue.get_nowait().unref()
        return True

    def start_cb(self):
        pass

    def end_cb(self):
        pass

    def h264_frame_cb(self, h264_frame):
        pass

    def display_frame(self, yuv_frame):
        info = yuv_frame.info()

        height, width = ( 
            info["raw"]["frame"]["info"]["height"],
            info["raw"]["frame"]["info"]["width"],
        )

        cv2_cvt_color_flag = {
            olympe.VDEF_I420: cv2.COLOR_YUV2BGR_I420,
            olympe.VDEF_NV12: cv2.COLOR_YUV2BGR_NV12,
        }[yuv_frame.format()]

        cv2frame = cv2.cvtColor(yuv_frame.as_ndarray(), cv2_cvt_color_flag)

        if not self.is_detecting:
            self.detected_targets = np.zeros(self.num_targets, dtype=bool)
        else: 
            results = self.decode(cv2frame)

        self.out.write(cv2frame)
        cv2.imshow("Frames via Olympe", cv2frame)
        cv2.waitKey(1)

    def run(self):
        main_thread = next(
            filter(lambda t: t.name == "MainThread", threading.enumerate())
        )
        while main_thread.is_alive():
            with self.flush_queue_lock:
                try:
                    yuv_frame = self.frame_queue.get(timeout=0.01)
                except queue.Empty:
                    continue
                try:
                    self.display_frame(yuv_frame)
                except Exception as e:
                    print(e)
                finally:
                    yuv_frame.unref()


class Drone:
    def __init__(self, drone_ip, num_targets, max_timestep, is_training=False):
        self.drone = olympe.Drone(drone_ip)
        self.drone.connect(retry=3)
        
        self.drone(GPSFixStateChanged(_policy = 'wait'))
        if not is_training: self._setup_camera()
        self.streamer = Streaming(self.drone, num_targets)
        self.streamer.start()

        self._takeoff()

        self.action = Action(self.drone)
        
        self.is_training = is_training
        self.num_targets = num_targets
        self.max_timestep = max_timestep
        self.timestep = 0
        self.visited_targets = np.zeros(self.num_targets, dtype=bool)
    
    def take_action(self, action):
        gps_reading = self.drone.get_state(GpsLocationChanged)
        battery = self.drone.get_state(BatteryStateChanged)["percent"]
        
        old_cell, new_cell, action_name = self.action.take_action(action)
        self.timestep += 1
        
        detected_targets = self._detect_targets(new_cell)
        
        reward = self._get_reward(detected_targets) 
        state = self._get_state(new_cell, detected_targets) 
        if self.timestep >= self.max_timestep or np.all(self.visited_targets):
            done = True
            self.visited_targets[:] = False
        else:
            done = False
        info = {
            "action": str(action_name), 
            "direction": "Cell " + str(old_cell) + " --> " + "Cell " + str(new_cell),
        }
        
        logger.info(
            f"{gps_reading['latitude']},{gps_reading['longitude']},{gps_reading['altitude']},"
            f"{action_name},{old_cell},{new_cell},{battery}"
        ) 
        return state, reward, done, info

    def reset(self):
        old_cell, new_cell = self.action.reset()
        detected_targets = self._detect_targets(new_cell)
        self.timestep = 0
        return self._get_state(new_cell, detected_targets)
    
    def _get_state(self, new_cell, detected_targets):
        # {t, cell_id, [I1, I2, I3, ..., In]}
        
        self.visited_targets[detected_targets] = True
        
        return np.concatenate(([self.timestep, new_cell], self.visited_targets)).astype(np.uint8)
    
    def _get_reward(self, detected_targets):
        reward_scale = 1.5
        num_new_targets = np.count_nonzero(
            detected_targets & (detected_targets != self.visited_targets)
        )
        return reward_scale * num_new_targets if num_new_targets > 0 else -1
        
    def _detect_targets(self, cell_id):
        self.streamer.is_detecting = True
        time.sleep(DETECTION_DELAY)
        print("Detected by streamer: ", self.streamer.detected_targets)
        detected_targets = self.streamer.detected_targets
        self.streamer.is_detecting = False
                
        return detected_targets

    def _setup_camera(self):
        assert self.drone(
            set_target(
                gimbal_id=0,
                control_mode="position",
                yaw_frame_of_reference="none",
                yaw=0.0,
                pitch_frame_of_reference="absolute",
                pitch=-90.0,
                roll_frame_of_reference="none",
                roll=0.0,
                )
            >> attitude(
                pitch_absolute=-90.0, _policy="wait", _float_tol=(1e-3, 1e-1)
                )
            ).wait(_timeout=20).success()
    
    def _takeoff(self):
        takeoff_success = self._success_if_takeoff()
        if not takeoff_success:
            print("Retrying taking off...")
            takeoff_success = self._success_if_takeoff()
    
    def _success_if_takeoff(self):
        return self.drone(
                FlyingStateChanged(state="hovering")
                | (TakeOff() & FlyingStateChanged(state="hovering"))
            ).wait(10).success()
        
    def _land(self):
        self.drone(PCMD(1, 0, 0, 0, 0, 0) >> FlyingStateChanged(state="hovering", _timeout=5)).wait()
        assert self.drone(Landing() >> FlyingStateChanged(state="landed")).wait().success()
        
    def __del__(self):
        self._land()
        self.streamer.stop()
        del self.streamer
        del self.action
        self.drone.disconnect()


class AnafiEnv(Env):
    def __init__(self, num_targets, max_timestep, drone_ip=DRONE_IP, is_training=False):
        super(AnafiEnv, self).__init__()
        
        self.num_targets = num_targets
        self.max_timestep = max_timestep
        self.begin(num_targets, max_timestep, is_training, drone_ip)
        
        self.action_space = spaces.Discrete(len(self.agent.action))
        self.observation_space = spaces.Box(
            low=np.array([0, 1] + num_targets*[0]), 
            high=np.array([max_timestep, 25] + num_targets*[1]), 
            dtype=np.uint8,
        )
    
    def begin(self, num_targets, max_timestep, is_training, drone_ip=DRONE_IP):
        self.agent = Drone(drone_ip, num_targets, max_timestep, is_training)
    
    def step(self, action):
        obs, reward, done, info = self.agent.take_action(action)
        return obs, reward, done, info
    
    def reset(self):
        return self.agent.reset()
    
    def render(self, mode='human'):
        pass
    
    def close(self):
        del self.agent


def disp_info(action, observation, reward, done, info):
    print("Action:", info["action"] + ",", info["direction"])
    print("State:", observation)
    print("Reward:", reward)
    print("================================================")

def deploy():
    env = AnafiEnv(num_targets=10, max_timestep=MAX_TIMESTEP, is_training=False)
    model = PPO.load(MODEL_PATH, env, verbose=1)
    observation = env.reset()
    while True:
        action, _state = model.predict(observation, deterministic=True)
        observation, reward, done, info = env.step(action)
        disp_info(action, observation, reward, done, info)

        if done:
            print("The episode has ended...")
            observation = env.reset()
            disp_info(action, observation, 0, done, info)
            break

    env.close()

if __name__ == "__main__":
    deploy()
