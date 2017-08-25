import user_tracker as ut
import face_detector as fd
import time as time
import sys
import cv2

user_name = '1'
save_face = False
default_save_path = './dataset/2/'
data_set_path = './dataset/data_' + user_name + '.txt'
rgb_video_path = './dataset/rgb_' + user_name + '.avi'
depth_video_path = './dataset/depth_' + user_name + '.avi'

user_tracker = ut.UserTracker(rgb_video_path,
                              depth_video_path,
                              data_set_path)
face_detector = fd.FaceDetector()


begining_time = 0
user_counter = 0
while user_tracker.frame_exist():
    face_detected, faces = face_detector.get_faces(user_tracker.get_current_frame())
    if save_face and face_detected:
        for face in faces:
            user_counter += 1
            save_path = default_save_path + str(user_counter) + '.jpg'
            # print save_path
            cv2.imwrite(save_path, face)
            cv2.waitKey(1)

    # print time.time() - begining_time
    user_tracker.show_frames()
    begining_time = time.time()