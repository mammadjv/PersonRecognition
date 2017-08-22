import user_tracker as ut
import FaceDetector as fd
import time as time
import sys

user_name = 'hossein'

data_set_path = './dataset/data_' + user_name + '.txt'
rgb_video_path = './dataset/rgb_' + user_name + '.avi'
depth_video_path = './dataset/depth_' + user_name + '.avi'

user_tracker = ut.UserTracker(rgb_video_path,
                              depth_video_path,
                              data_set_path)
face_detector = fd.FaceDetector(user_name)

begining_time = 0
while user_tracker.frame_exist():
    face_detector.get_faces(user_tracker.get_current_frame(), save_face=False)
    # print time.time() - begining_time
    user_tracker.show_frames()
    begining_time = time.time()