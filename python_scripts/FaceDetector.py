import cv2
import time

class FaceDetector:

    def __init__(self):
        print "Face Detector module created"

    def crop_face(self, head_joints, frames):
        x_begin , y_begin = head_joints[0].get_x_image(), head_joints[0].get_y_image()
        x_end, y_end = head_joints[1].get_x_image(), head_joints[1].get_y_image()

        rgb_frame, depth_frame = frames[0].copy(), frames[1].copy()
        cv2.circle(rgb_frame, (x_begin,y_begin), 1, (0, 255, 0), 3)
        cv2.circle(rgb_frame, (x_end, y_end), 1, (0, 255, 0), 3)
        rgb_croped = rgb_frame[y_begin:y_end, x_begin:x_end]
        cv2.imshow('crop_rgb', rgb_frame)
        cv2.waitKey(1)
        return rgb_croped

    def get_faces(self, frame):
        if frame.get_number_of_persons() == 0:
            return None
        ## init array of faces
        faces = []
        for person in frame.get_persons():
            head_joints = []
            for joint in person.get_joints():
                if joint.get_joint_name() == 'head':
                  head_joints.append(joint)
                if joint.get_joint_name() == 'torso':
                   head_joints.append(joint)
                if len(head_joints) == 2:
                   ## detect faces in image
                   faces.append(self.crop_face(head_joints, frame.get_frames()))
                   break