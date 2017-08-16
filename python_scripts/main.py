import user_tracker as ut

user_tracker = ut.UserTracker("./dataset/rgb_2.avi",
                              "./dataset/depth_2.avi",
                              "./dataset/data_2.txt")

while user_tracker.frame_exist():
    user_tracker.show_frames()
