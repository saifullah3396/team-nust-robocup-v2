#!/usr/bin/python

import json
import rospy
import socket as sc
from socket import error
import select
from memory_data_publisher import MemoryDataPublisher
from publishers import ImagePublisher, MessagePublisher, \
    KnownLandmarkPublisher, UnknownLandmarkPublisher, PFStatePublisher, \
    FootstepsPublisher
from comm_msg_types import CommMessageTypes
from std_msgs.msg import String

class DataHandler():
    def __init__(self):
        self._data_subscriber = rospy.Subscriber("/team_nust_nao_data", String, self.parse_data)
        self._image_subscriber = rospy.Subscriber("/team_nust_nao_image", String, self.parse_image)
        self._last_heart_beat_at = None
        self._memory_data_publisher = MemoryDataPublisher()
        self._log_message_publisher = MessagePublisher('log_messages')
        self._unknown_landmark_publisher = UnknownLandmarkPublisher('obs_unknown_landmarks')
        self._known_landmark_publisher = KnownLandmarkPublisher('obs_known_landmarks')
        self._particle_filter_state_publisher = PFStatePublisher('pf_states')
        self._footsteps_publisher = FootstepsPublisher('cmd_footsteps')
        self._image_publisher = ImagePublisher('nao_image')
        
    def parse_image(self, image):
        image = image.data
        if image == None:
          return
        try:
          self._image_publisher.handle_image_msg(image)
          self._image_publisher.publish()
        except AssertionError as e:
            pass
  
    def parse_data(self, data):
        json_data = json.loads(data.data)
        for msg_type in json_data:
          try:
              if int(msg_type) == CommMessageTypes.MEMORY:
                  self._memory_data_publisher.handle_memory_data_msg(json_data[msg_type])
                  self._memory_data_publisher.publish()
              elif int(msg_type) == CommMessageTypes.LOG_TEXT:
                  self._log_message_publisher.handle_log_message(json_data[msg_type])
                  self._log_message_publisher.publish()
              elif int(msg_type) == CommMessageTypes.UNKNOWN_LANDMARKS:
                  self._unknown_landmark_publisher.handle_landmark_data(json_data[msg_type])
                  self._unknown_landmark_publisher.publish()
              elif int(msg_type) == CommMessageTypes.KNOWN_LANDMARKS:
                  self._known_landmark_publisher.handle_landmark_data(json_data[msg_type])
                  self._known_landmark_publisher.publish()
              elif int(msg_type) == CommMessageTypes.PF_STATES:
                  self._particle_filter_state_publisher.handle_particle_data(json_data[msg_type])
                  self._particle_filter_state_publisher.publish()
              elif int(msg_type) == CommMessageTypes.FOOTSTEPS:
                  self._footsteps_publisher.handle_footsteps_data(json_data[msg_type])
                  self._footsteps_publisher.publish()
          except AssertionError as e:
              pass

    def handle_heart_beat_msg(self):
        self._last_heart_beat_at = rospy.Time.now()
