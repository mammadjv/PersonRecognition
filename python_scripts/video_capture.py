import cv2


class VideoCapture:
    def __init__(self, rgb_video_file, depth_video_file):
        self.rgb_cap = cv2.VideoCapture(rgb_video_file)
        self.depth_cap = cv2.VideoCapture(depth_video_file)

    def read_frame(self):
        if self.rgb_cap.isOpened() and self.depth_cap.isOpened():
            rgb_ret, rgb_frame = self.rgb_cap.read()
            depth_ret, depth_frame = self.depth_cap.read()
            if rgb_ret is True and depth_ret is True:
                return True, rgb_frame, depth_frame
            else:
                return False, None, None
