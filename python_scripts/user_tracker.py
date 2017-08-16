import video_capture
import frame_creator
import cv2
import sys
import numpy as np

class UserTracker:
    def __init__(self, rgb_video, depth_video, datafile):
        self.video_capture = video_capture.VideoCapture(rgb_video, depth_video)
        self.frame_creator = frame_creator.FrameCreator(datafile)
        self.frame_existence = False
        self.current_frame_id = 1
        self.users = []
        self.frame_creator.parse()

    def frame_exist(self):
        self.current_frame_id = self.current_frame_id + 1
        self.frame_existence, rgb_frame, depth_frame = self.video_capture.read_frame()

        if self.frame_existence:
            if self.current_frame_id > len(self.frame_creator.frames)-1:
                self.frame_creator.add_empty_frame()
            self.frame_creator.frames[self.current_frame_id].set_images(rgb_frame, depth_frame)
        return self.frame_existence

    def get_current_frame(self):
        return frame_creator.frames[self.current_frame_id]

    def show_frames(self):
        rgb_frame , depth_frame = self.frame_creator.frames[self.current_frame_id].get_frames()
        cv2.imshow('rgb_frame', rgb_frame)
        cv2.imshow('depth_frame', depth_frame)
        cv2.waitKey(1)