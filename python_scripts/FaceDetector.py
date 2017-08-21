import cv2
import time
import math

class FaceDetector:

    def __init__(self):
        print "Face Detector module created"

    def crop_face(self, head_joints, shoulder_joints, frames):
        length_threshold = 10
        x_begin, y_begin = head_joints[0].get_x_image(), head_joints[0].get_y_image()
        x_end, y_end = head_joints[1].get_x_image(), head_joints[1].get_y_image()

        x_left_shoulder, y_left_shoulder = shoulder_joints[0].get_x_image(), shoulder_joints[0].get_y_image()
        x_right_shoulder, y_right_shoulder = shoulder_joints[1].get_x_image(), shoulder_joints[1].get_y_image()

        point_step_correction = 10
        if math.fabs(y_begin-y_end) < length_threshold:
            point_step_correction = 5

        x_begin -= point_step_correction
        y_begin += 2*point_step_correction
        x_end = x_begin

        rgb_frame, depth_frame = frames[0].copy(), frames[1].copy()
        cv2.circle(rgb_frame, (x_begin,y_begin), 1, (0, 255, 0), 3)
        cv2.circle(rgb_frame, (x_end, y_end), 1, (0, 255, 0), 3)

        cv2.circle(rgb_frame, (x_left_shoulder, y_left_shoulder), 1, (255, 0, 0), 3)
        cv2.circle(rgb_frame, (x_right_shoulder, y_right_shoulder), 1, (0, 0, 225), 3)

        rgb_croped = rgb_frame[y_begin:y_end, x_begin:x_end]
        cv2.imshow('crop_rgb', rgb_frame)
        cv2.waitKey(3)
        return rgb_croped

    def get_faces(self, frame):
        if frame.get_number_of_persons() == 0:
            return None
        ## init array of faces
        faces = []
        for person in frame.get_persons():
            face_exists, head_joints, shoulder_joints = person.face_and_head_joints_exists_for_person()
            if not face_exists:
                continue
            print frame.get_frame_id()
            faces.append(self.crop_face(head_joints, shoulder_joints, frame.get_frames()))