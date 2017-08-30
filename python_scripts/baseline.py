import user_tracker as ut
import time as time
import sys
import cv2
import feature_combiner as fc
from sklearn.metrics import accuracy_score
from sklearn.metrics import confusion_matrix



name_list = ['0', '1', '2', '3']
data = []
# prepare training data 
for name in name_list:
    data_set_path = 'dataset/data_' + name + '.txt'
    rgb_video_path = 'dataset/rgb_' + name + '.avi'
    depth_video_path = 'dataset/depth_' + name + '.avi'
    #
    user_tracker = ut.UserTracker(rgb_video_path,
                              depth_video_path,
                              data_set_path)
    person_all_points = []
    while user_tracker.frame_exist():
        frame = user_tracker.get_current_frame()
        if frame.get_persons() != []:
            person = frame.get_persons()[0]
            person_joints = range(15)
            for joint in person.get_joints():
                joint_xyz = [joint.get_x_world(), joint.get_y_world(), joint.get_z_world()]
                joint_name = joint.get_joint_name()
                if joint_name == 'head':
                    person_joints[0] = list(joint_xyz)
                if joint_name == 'neck':
                    person_joints[1] = list(joint_xyz)
                if joint_name == 'torso':
                    person_joints[2] = list(joint_xyz)
                if joint_name == 'right_shoulder':
                    person_joints[3] = list(joint_xyz)
                if joint_name == 'right_elbow':
                    person_joints[4] = list(joint_xyz)
                if joint_name == 'right_hand':
                    person_joints[5] = list(joint_xyz)
                if joint_name == 'left_shoulder':
                    person_joints[6] = list(joint_xyz)
                if joint_name == 'left_elbow':
                    person_joints[7] = list(joint_xyz)
                if joint_name == 'left_hand':
                    person_joints[8] = list(joint_xyz)
                if joint_name == 'right_hip':
                    person_joints[9] = list(joint_xyz)
                if joint_name == 'right_knee':
                    person_joints[10] = list(joint_xyz)
                if joint_name == 'right_foot':
                    person_joints[11] = list(joint_xyz)
                if joint_name == 'left_hip':
                    person_joints[12] = list(joint_xyz)
                if joint_name == 'left_knee':
                    person_joints[13] = list(joint_xyz)
                if joint_name == 'left_foot':
                    person_joints[14] = list(joint_xyz)
            person_all_points.append(list(person_joints))
            # user_tracker.show_frames()
    data.append(list(person_all_points))


feature_combiner = fc.FeatureCombiner(data)
mean_list, std_list = feature_combiner.train_joints()


# test the model
y_true = []
y_pred = []
for name in name_list:
    data_set_path = 'dataset/data_' + name + '.txt'
    rgb_video_path = 'dataset/rgb_' + name + '.avi'
    depth_video_path = 'dataset/depth_' + name + '.avi'
    #
    user_tracker = ut.UserTracker(rgb_video_path,
                              depth_video_path,
                              data_set_path)
    while user_tracker.frame_exist():
        frame = user_tracker.get_current_frame()
        if frame.get_persons() != []:
            person = frame.get_persons()[0]
            person_joints = range(15)
            for joint in person.get_joints():
                joint_xyz = [joint.get_x_world(), joint.get_y_world(), joint.get_z_world()]
                joint_name = joint.get_joint_name()
                if joint_name == 'head':
                    person_joints[0] = list(joint_xyz)
                if joint_name == 'neck':
                    person_joints[1] = list(joint_xyz)
                if joint_name == 'torso':
                    person_joints[2] = list(joint_xyz)
                if joint_name == 'right_shoulder':
                    person_joints[3] = list(joint_xyz)
                if joint_name == 'right_elbow':
                    person_joints[4] = list(joint_xyz)
                if joint_name == 'right_hand':
                    person_joints[5] = list(joint_xyz)
                if joint_name == 'left_shoulder':
                    person_joints[6] = list(joint_xyz)
                if joint_name == 'left_elbow':
                    person_joints[7] = list(joint_xyz)
                if joint_name == 'left_hand':
                    person_joints[8] = list(joint_xyz)
                if joint_name == 'right_hip':
                    person_joints[9] = list(joint_xyz)
                if joint_name == 'right_knee':
                    person_joints[10] = list(joint_xyz)
                if joint_name == 'right_foot':
                    person_joints[11] = list(joint_xyz)
                if joint_name == 'left_hip':
                    person_joints[12] = list(joint_xyz)
                if joint_name == 'left_knee':
                    person_joints[13] = list(joint_xyz)
                if joint_name == 'left_foot':
                    person_joints[14] = list(joint_xyz)
            who, log_prob = feature_combiner.test_joints(person_joints, mean_list, std_list)
            y_pred.append(who)
            y_true.append(int(name))
            # user_tracker.show_frames()
    data.append(list(person_all_points))

print('Accuracy = ' + str(accuracy_score(y_true, y_pred)))

print(confusion_matrix(y_true, y_pred))

