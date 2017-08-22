import user_tracker as ut
import FaceDetector as fd
import time as time


user_tracker = ut.UserTracker("./dataset/rgb_mammad.avi",
                              "./dataset/depth_mammad.avi",
                              "./dataset/data_mammad.txt")
face_detector = fd.FaceDetector()

begining_time = 0
while user_tracker.frame_exist():
    face_detector.get_faces(user_tracker.get_current_frame())
    # print time.time() - begining_time
    user_tracker.show_frames()
    begining_time = time.time()