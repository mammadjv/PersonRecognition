import user_tracker as ut

user_tracker = ut.UserTracker("/home/mohammad/catkin_ws/oniFiles/dataset/rgb_1.avi",
                              "/home/mohammad/catkin_ws/oniFiles/dataset/depth_1.avi",
                              "/home/mohammad/catkin_ws/oniFiles/dataset/data_1.txt")

while user_tracker.frame_exist():
    user_tracker.show_frames()
