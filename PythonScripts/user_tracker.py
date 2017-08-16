import video_capture
import file_streamer
import cv2


class UserTracker:
    def __init__(self, rgb_video, depth_video, datafile):
        self.video_capture = video_capture.VideoCapture(rgb_video, depth_video)
        self.file_streamer = file_streamer.FileStreamer(datafile)
        self.frame_existence = False
        self.rgb_frame = self.depth_frame = None
        self.users = []

    def frame_exist(self):
        self.frame_existence, self.rgb_frame, self.depth_frame = self.video_capture.read_frame()
        return self.frame_existence

    def get_rgb_frame(self):
        return self.rgb_frame

    def get_depth_frame(self):
        return self.depth_frame

    def get_frames(self):
        return self.rgb_frame, self.depth_frame

    def show_frames(self):
        cv2.imshow('rgb_frame', self.rgb_frame)
        cv2.imshow('depth_frame', self.depth_frame)
        cv2.waitKey(1)
