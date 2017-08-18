import user_tracker as ut
import FaceDetector as fd

user_tracker = ut.UserTracker("./dataset/rgb_2.avi",
                              "./dataset/depth_2.avi",
                              "./dataset/data_2.txt")
face_detector = fd.FaceDetector()

while user_tracker.frame_exist():
    face_detector.get_faces(user_tracker.get_current_frame())
    user_tracker.show_frames()
