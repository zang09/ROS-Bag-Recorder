#!/usr/bin/env python

import rospy
import rosnode
from std_srvs.srv import Trigger, TriggerResponse
from bag_recorder.srv import *

import os
import subprocess

class BagRecorder():
    def __init__(self):
        self.save_directory_service = rospy.Service('/bag_recorder/save_directory', saveDirectory, self.save_directory)
        self.start_recording_service = rospy.Service('/bag_recorder/start_recording', Trigger, self.start_recording)
        self.stop_recording_service = rospy.Service('/bag_recorder/stop_recording', Trigger, self.stop_recording)
        self.stop_recording_service = rospy.Service('/bag_recorder/toggle_recording', Trigger, self.toggle_recording)
        
        self.process = None
        self.recording = False

        self.output_directory = rospy.get_param('/bag_recorder/output_directory', '~/rosbag/')

        self.topics = rospy.get_param('/bag_recorder/topics', [])
        if not self.topics:
            rospy.logerr('No Topics Specified.')

        self.command = ['rosrun', 'rosbag', 'record', '-e', '--split', '--size=1024'] + self.topics + ['__name:=bag_recorder_myrecorder']

        rospy.loginfo('Bag Recorder Started')

    def save_directory(self, req):
        self.output_directory = req.path
        return saveDirectoryResponse(True)

    def toggle_recording(self, req):
        if self.recording:
            return self.stop_recording(req)
        else:
            return self.start_recording(req)

    def start_recording(self, req):
        if self.recording:
            rospy.logerr('Already Recording')
            return TriggerResponse(False, 'Already Recording')
        
        #print(self.command)
        self.process = subprocess.Popen(self.command, cwd=self.output_directory)
        self.recording = True
        rospy.loginfo('Started recorder, PID %s' % self.process.pid)
        return TriggerResponse(True, 'Started recorder, PID %s' % self.process.pid)

    def stop_recording(self, req):
        if not self.recording:
            rospy.logerr('Not Recording')
            return TriggerResponse(False, 'Not Recording')

        rosnode.kill_nodes(['/bag_recorder_myrecorder'])

        self.process = None
        self.recording = False

        rospy.loginfo('Stopped Recording')
        return TriggerResponse(True, 'Stopped Recording')


if __name__ == "__main__":
    rospy.init_node('bag_recorder')
    BagRecorder()
    rospy.spin()
