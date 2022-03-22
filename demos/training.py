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
import re
from collections import OrderedDict

import PySimpleGUI as sg

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

from pynput.keyboard import Listener, Key, KeyCode
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

DRONE_IP = os.environ.get("DRONE_IP", "10.202.0.1")
DRONE_MEDIA_PORT = os.environ.get("DRONE_MEDIA_PORT", "80")

ANAFI_URL = "http://{}/".format(DRONE_IP)
ANAFI_MEDIA_API_URL = ANAFI_URL + "api/v1/media/medias/"

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
#         direction = str(self.Move(action)).split(".")[1].capitalize()
#         return "Moving " + direction if "hover" not in direction.lower() else "Hovering"
    
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
        altitude = 6.0
        dlong = 6.8e-5 # in degrees == 5 meters along x-axis (forward[+]-backward[-])
        dlat = 7.2e-5 # in degrees == 8 meters along y-axis (left[+]-right[-])
        
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

class Drone:
    def __init__(self, drone_ip, num_targets, max_timestep, is_training=False):
        self.drone = olympe.Drone(drone_ip)
        self.drone.connect()
        
        self.drone(GPSFixStateChanged(_policy = 'wait'))
        self._takeoff()
        if not is_training: self._setup_camera()

        self.action = Action(self.drone)
        
        self.is_training = is_training
        self.num_targets = num_targets
        self.max_timestep = max_timestep
        self.timestep = 0
        self.visited_targets = np.zeros(self.num_targets, dtype=bool)
        self.target_positions = np.zeros(self.num_targets, dtype=np.uint8)
    
    def take_action(self, action):
        old_cell, new_cell, action_name = self.action.take_action(action)
        self.timestep += 1
        
        Simulation.cease_targets()
        time.sleep(0.1)
        detected_targets = self._detect_targets(new_cell)
        
        reward = self._get_reward(detected_targets) # !!! _get_reward must come before self.visited_targets is changed in _get_state
        state = self._get_state(new_cell, detected_targets) 
        if self.timestep >= self.max_timestep or np.all(self.visited_targets):
            done = True
            self.visited_targets[:] = False
        else:
            done = False
        info = {
            "action": str(action_name), 
            "direction": "Cell " + str(old_cell) + " --> " + "Cell " + str(new_cell),
            "positions": self.target_positions
        }
        # print(state)
        # print(reward)
        return state, reward, done, info

    def reset(self):
        old_cell, new_cell = self.action.reset()
        detected_targets = self._detect_targets(new_cell)
        self.timestep = 0
        Simulation.move_targets()
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
        self.update_moving_target_positions()
        return self.target_positions == cell_id
        
        if self.is_training:
            positions = np.genfromtxt('target_positions.csv', delimiter=',', skip_header=1, dtype=np.uint8)
            return positions[:,1] == cell_id
        
        detected_targets = np.zeros(self.num_targets, dtype=bool)

        img = self._take_photo()
        if img is None:
            return detected_targets
        
        for result in pyzbar.decode(img):
            idx = int(result.data) - 1
            try:
                detected_targets[idx] = True
            except (ValueError, IndexError):
                pass
                
        return detected_targets

    def update_moving_target_positions(self):
        Simulation.update_moving_target_positions(self.target_positions)
        
    def _setup_camera(self):
        assert self.drone.media_autoconnect
        self.drone.media.integrity_check = True
        is_indexed = False
        while not is_indexed:
            is_indexed = self.drone.media(
                indexing_state(state="indexed")
            ).wait(_timeout=5).success()
        
        self.drone(set_camera_mode(cam_id=0, value="photo")).wait()

        assert self.drone(
            set_photo_mode(
                cam_id=0,
                mode="single",
                format= "rectilinear",
                file_format="jpeg",
                # the following are ignored in photo single mode
                burst="burst_14_over_1s",
                bracketing="preset_1ev",
                capture_interval=5.0,
            )
        ).wait().success()

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
    
    def _take_photo(self):
        photo_saved = self.drone(photo_progress(result="photo_saved", _policy="wait"))
        self.drone(take_photo(cam_id=0)).wait()
        
        photo_taken = False
        tries = 0
        while not photo_taken:
            tries += 1
            if tries > 3:
#                 assert False, "take_photo timedout"
                print("take_photo timedout")
                return None
            photo_taken = photo_saved.wait(_timeout=5).success()
            
        # get the bytes of the image
        media_id = photo_saved.received_events().last().args["media_id"]
        for _ in range(5):
            media_info_response = requests.get(ANAFI_MEDIA_API_URL + media_id, timeout=10)
            if media_info_response.status_code == 200:
                break
        try:
            media_info_response.raise_for_status()
        except requests.exceptions.HTTPError as err:
            print(err)
            return None
        
        resource = media_info_response.json()["resources"][0]
        image_response = requests.get(ANAFI_URL + resource["url"], stream=True)
        image_response.raise_for_status()
        
        img = Image.open(BytesIO(image_response.content))
        
        # delete the image stored on the drone
        photo_deleted = False
        delete_tries = 0
        while not photo_deleted:
            delete_tries += 1
            if delete_tries > 3:
#                 assert False, "Failed to delete media {} {}".format(media_id, delete.explain())
                print("Failed to delete media {} {}".format(media_id, delete.explain()))
                break
            delete = delete_media(media_id, _timeout=10)
            photo_deleted = self.drone.media(delete).wait().success()
        
        return img
    
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
        self.drone.disconnect()
        del state
        del reward
        del action

class Simulation:
    sphinx = jsonrpclib.Server('http://127.0.0.1:8383')
    
    def __init__(self):
        pass
    
    @staticmethod
    def disable_battery():
        Simulation.sphinx.SetParam(machine='anafi4k',
                                   object='lipobattery/lipobattery',
                                   parameter='discharge_speed_factor',
                                   value='0')
    
    @staticmethod
    def reset_world():
        Simulation.sphinx.TriggerAction(machine='world',
                                        object='fwman/fwman',
                                        action='world_reset_all')
    
    def find_distance(x, y):
        x = x - y[:,np.newaxis]
        x = x**2
        x = x.sum(axis=2)
        x = np.sqrt(x)
        return x

    # check if the target overlaps with another target or border
    @classmethod
    def is_overlapping(cls, x, y=None, limit=0.8):
        # check with the borders
        borders_parallel_y = np.array([12.5, 7.5, 2.5, -2.5, -7.5, -12.5])
        x1 = abs(x[:,0] - borders_parallel_y[:,np.newaxis]) < 0.4

        borders_parallel_x = np.array([20, 12, 4, -4, -12, -20])
        x2 = abs(x[:,1] - borders_parallel_x[:,np.newaxis]) < 0.4
        
        # check if outside the boundaries
        x3 = (
            (x[:, 0] > borders_parallel_y.max()) | (x[:, 0] < borders_parallel_y.min()) | 
            (x[:, 1] > borders_parallel_x.max()) | (x[:, 1] < borders_parallel_x.min())
        )

        # check with another target
        if y is None: y = x
        x = cls.find_distance(x, y) < limit
        x = np.triu(x, k=1)

        x = np.vstack((x, x1, x2, x3))
        x = np.any(x, axis=0)
        return x
    
    def coord_to_cellid(locs):
        cell_boundaries = [
           (locs[:,0] > 7.5) & (locs[:,1] > 12),
           (locs[:,0] > 7.5) & (locs[:,1] > 4),
           (locs[:,0] > 7.5) & (locs[:,1] > -4),
           (locs[:,0] > 7.5) & (locs[:,1] > -12),
           (locs[:,0] > 7.5) & (locs[:,1] > -20),
           (locs[:,0] > 2.5) & (locs[:,1] > 12),
           (locs[:,0] > 2.5) & (locs[:,1] > 4),
           (locs[:,0] > 2.5) & (locs[:,1] > -4),
           (locs[:,0] > 2.5) & (locs[:,1] > -12),
           (locs[:,0] > 2.5) & (locs[:,1] > -20),
           (locs[:,0] > -2.5) & (locs[:,1] > 12),
           (locs[:,0] > -2.5) & (locs[:,1] > 4),
           (locs[:,0] > -2.5) & (locs[:,1] > -4),
           (locs[:,0] > -2.5) & (locs[:,1] > -12),
           (locs[:,0] > -2.5) & (locs[:,1] > -20),
           (locs[:,0] > -7.5) & (locs[:,1] > 12),
           (locs[:,0] > -7.5) & (locs[:,1] > 4),
           (locs[:,0] > -7.5) & (locs[:,1] > -4),
           (locs[:,0] > -7.5) & (locs[:,1] > -12),
           (locs[:,0] > -7.5) & (locs[:,1] > -20),
           (locs[:,0] > -12.5) & (locs[:,1] > 12),
           (locs[:,0] > -12.5) & (locs[:,1] > 4),
           (locs[:,0] > -12.5) & (locs[:,1] > -4),
           (locs[:,0] > -12.5) & (locs[:,1] > -12),
           (locs[:,0] > -12.5) & (locs[:,1] > -20),
        ]
        cell_ids = np.arange(25) + 1
        return np.select(cell_boundaries, cell_ids)
    
    @classmethod
    def gen_targets_pos(cls, num_targets):

        num_mobile = 3
        num_fixed = num_targets-num_mobile

        mu_x_fixed = 5.5
        variance_x_fixed = 7
        mu_y_fixed = -10 
        variance_y_fixed = 10
        distribution_fixed = multivariate_normal(
            [mu_x_fixed, mu_y_fixed], 
            [[variance_x_fixed, 0], [0, variance_y_fixed]]
        )

        mu_x_mobile = -7.5
        variance_x_mobile = 3
        mu_y_mobile = 5
        variance_y_mobile = 30
        distribution_mobile = multivariate_normal(
            [mu_x_mobile, mu_y_mobile],
            [[variance_x_mobile, 0], [0, variance_y_mobile]]
        )

        locs = np.empty((num_targets,2))
        locs[:num_fixed] = distribution_fixed.rvs(size=num_fixed)
        locs[num_fixed:] = distribution_mobile.rvs(size=num_mobile)

        overlapping_targets = Simulation.is_overlapping(locs)
        while np.any(overlapping_targets):
            num_fixed_overlaps = np.count_nonzero(overlapping_targets[:num_fixed])
            num_mobile_overlaps = np.count_nonzero(overlapping_targets[num_fixed:])
            locs[:num_fixed][overlapping_targets[:num_fixed]] = distribution_fixed.rvs(size=num_fixed_overlaps)
            locs[num_fixed:][overlapping_targets[num_fixed:]] = distribution_mobile.rvs(size=num_mobile_overlaps)
            overlapping_targets = Simulation.is_overlapping(locs)
        
        np.savetxt('../comm/positions.csv', 
                   np.hstack((np.arange(len(locs))[:,np.newaxis] + 1, locs)),
                   fmt='%.4f',
                   delimiter=',',
                  )
        
        return cls.coord_to_cellid(locs) # , locs
    
    @classmethod
    def update_moving_target_positions(cls, arr, ids=[8, 9, 10]):
        ids = np.array(ids)
        for i in ids:
            locs = np.loadtxt("../comm/position_" + str(i) + ".csv", delimiter=',')
            arr[i-1] = cls.coord_to_cellid(locs[np.newaxis,:])
            
        return arr
    
    @staticmethod
    def cease_targets():
        f = open("../comm/toggle_movement.txt", "w")
        f.write("0")
        f.close()
    
    @staticmethod
    def move_targets():
        f = open("../comm/toggle_movement.txt", "w")
        f.write("1")
        f.close()
        
    @staticmethod
    def reset_targets(release=False):
        f = open("../comm/reset_position.txt", "w")
        if not release:
            f.write("1\n")
        else:
            f.write("0\n")
        f.close()

class AnafiEnv(Env):
    def __init__(self, num_targets, max_timestep, drone_ip="10.202.0.1", is_training=False):
        super(AnafiEnv, self).__init__()
        
        Simulation.disable_battery()
        Simulation.cease_targets()
        
        self.num_targets = num_targets
        self.max_timestep = max_timestep
        self.begin(num_targets, max_timestep, is_training, drone_ip)
        
        self.action_space = spaces.Discrete(len(self.agent.action))
        self.observation_space = spaces.Box( # {t, cell_id, [I1, I2, I3, ..., In]}
            low=np.array([0, 1] + num_targets*[0]), 
            high=np.array([max_timestep, 25] + num_targets*[1]), 
            dtype=np.uint8,
        )
        
        Simulation.move_targets()
    
    def begin(self, num_targets, max_timestep, is_training, drone_ip="10.202.0.1"):
        self.agent = Drone(drone_ip, num_targets, max_timestep, is_training)
    
    def step(self, action):
        obs, reward, done, info = self.agent.take_action(action)
        Simulation.move_targets()
        return obs, reward, done, info
    
    def reset(self):
        Simulation.cease_targets()
        self.agent.target_positions = Simulation.gen_targets_pos(self.num_targets)
        Simulation.reset_targets()
        time.sleep(0.1)
        Simulation.reset_targets(release=True)
        return self.agent.reset()
    
    def render(self, mode='human'):
        pass
    
    def close(self):
        Simulation.cease_targets()
        del self.agent

class SaveOnBestTrainingRewardCallback(BaseCallback):
    """
    Callback for saving a model (the check is done every ``check_freq`` steps)
    based on the training reward (in practice, we recommend using ``EvalCallback``).

    :param check_freq:
    :param log_dir: Path to the folder where the model will be saved.
      It must contains the file created by the ``Monitor`` wrapper.
    :param verbose: Verbosity level.
    """
    def __init__(self, check_freq: int, log_dir: str, verbose: int = 1):
        super(SaveOnBestTrainingRewardCallback, self).__init__(verbose)
        self.check_freq = check_freq
        self.log_dir = log_dir
        self.save_path = os.path.join(log_dir, 'best_model')
        self.best_mean_reward = -np.inf
        self.progress_dir = os.path.join(log_dir, 'progress')
        os.makedirs(self.progress_dir, exist_ok=True)

    def _init_callback(self) -> None:
        pass
#         # Create folder if needed
#         if self.save_path is not None:
#             os.makedirs(self.save_path, exist_ok=True)

    def _on_step(self) -> bool:
        if self.n_calls % 4096 == 0:
            self.model.save(os.path.join(self.progress_dir, 'model_' + str(self.n_calls)))
            
        if self.n_calls % self.check_freq == 0:

            # Retrieve training reward
            x, y = ts2xy(load_results(self.log_dir), 'timesteps')
            if len(x) > 0:
                # Mean training reward over the last 100 episodes
                mean_reward = np.mean(y[-100:])
                if self.verbose > 0:
                    print(f"Num timesteps: {self.num_timesteps}")
                    print(f"Best mean reward: {self.best_mean_reward:.2f} - Last mean reward per episode: {mean_reward:.2f}")

                # New best model, you could save the agent here
                if mean_reward > self.best_mean_reward:
                    self.best_mean_reward = mean_reward
                    # Example for saving best model
                    if self.verbose > 0:
                        print(f"Saving new best model to {self.save_path}")
                    self.model.save(self.save_path)
        
        return True

def ping(host):
    # Building the command. Ex: "ping -c 1 google.com"
    command = ['ping', '-c', '1', host]

    return subprocess.call(command) == 0

def train():
    # Create log dir
    log_dir = "logs/"
    os.makedirs(log_dir, exist_ok=True)
    run = len([filename for filename in os.listdir(log_dir) if filename.endswith("monitor.csv")]) + 1
    monitor_file = os.path.join(log_dir, str(run))

    env = AnafiEnv(num_targets=10, max_timestep=20, is_training=True)
    env = Monitor(env, monitor_file)
    env = DummyVecEnv([lambda: env])

    callback = SaveOnBestTrainingRewardCallback(check_freq=1024, log_dir=log_dir)

    # model = PPO("MlpPolicy", env, n_steps=2048, verbose=1, tensorboard_log=log_dir)
    # model = PPO.load(os.path.join(log_dir, str(run-1) + "_run"), env)
    model = PPO.load(os.path.join(log_dir, "best_model"), env)
    model.learn(total_timesteps=32_768, callback=callback, tb_log_name="PPO_" + str(run), reset_num_timesteps=False)
    model.save(os.path.join(log_dir, str(run) + "_run"))

if __name__ == "__main__":
    # print(ping("10.202.0.1"))
    train()
