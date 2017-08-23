import cv2
import time
import math
import numpy as np


class FaceDetector:

    def __init__(self):
        print "Face Detector module created"

    def crop_face(self, head_joints, shoulder_joints, frame):
        length_threshold = 10
        x_head, y_head = head_joints[0].get_x_image(), head_joints[0].get_y_image()
        x_neck, y_neck = head_joints[1].get_x_image(), head_joints[1].get_y_image()

        x_left_shoulder, y_left_shoulder = shoulder_joints[0].get_x_image(), shoulder_joints[0].get_y_image()
        x_right_shoulder, y_right_shoulder = shoulder_joints[1].get_x_image(), shoulder_joints[1].get_y_image()

        point_step_correction = 10
        if math.fabs(y_head-y_neck) < length_threshold:
            point_step_correction = 5

        x_head -= point_step_correction
        y_head += 2*point_step_correction
        x_neck = x_head

        rgb_frame, depth_frame = frame.get_rgb_frame(), frame.get_depth_frame()
        #cv2.circle(rgb_frame, (x_head,y_head), 1, (0, 255, 0), 3)
        #cv2.circle(rgb_frame, (x_neck, y_neck), 1, (0, 255, 0), 3)

        #cv2.circle(rgb_frame, (x_left_shoulder, y_left_shoulder), 1, (255, 0, 0), 3)
        #cv2.circle(rgb_frame, (x_right_shoulder, y_right_shoulder), 1, (0, 0, 225), 3)

        radius = math.sqrt(math.fabs(math.pow(x_head - x_neck, 2) + math.pow(y_head - y_neck, 2)))

        if radius < 10 or math.fabs(y_head - y_neck) < 5:
            return None

        print radius, x_head , y_head
        x_begin = int(max(x_head - radius, 0))
        x_end = int(min(x_head + radius, 639))
        y_begin = int(max(y_head - radius, 0))
        y_end = int(min(y_head + radius, 479))

        print y_begin, y_end, x_begin, x_end
        rgb_croped = rgb_frame[y_begin:y_end, x_begin:x_end]
        return rgb_croped

    def get_faces(self, frame):
        if frame.get_number_of_persons() == 0:
            return False, None
        # init array of faces
        faces = []
        for person in frame.get_persons():
            face_exists, head_joints, shoulder_joints = person.face_and_head_joints_exists_for_person()
            if not face_exists:
                continue
            # print frame.get_frame_id()
            detected_face = self.crop_face(head_joints, shoulder_joints, frame)
            if detected_face == None:
                continue
            cv2.imshow('croped_face', detected_face)
            faces.append(detected_face)
        return True, faces