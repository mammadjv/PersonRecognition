import pandas as pd
import numpy as np
import cv2


class FrameCreator:
    def __init__(self, datafile):
        self.data = datafile
        self.frames = []

    def parse(self):
        df = pd.read_csv(self.data, sep='\n', header=None, index_col=None)
        last_frame_id = 1
        for index in range(0, len(df)):
            line = df.iloc[index].values[0]
            if line.startswith('id'):
                pass
            if not line.startswith('id'):
                frame_temp_id = int(line.split(' ')[0])
                frame_temp_number_of_persons = int(line.split(' ')[1])
                frame_temp = Frame(frame_temp_id, frame_temp_number_of_persons)
                for i in range(last_frame_id + 1, frame_temp_id):
                    self.frames.append(Frame(i, 0))
                last_frame_id = frame_temp_id
                for i in range(0, frame_temp_number_of_persons):
                    user_line = df.iloc[index + 1 + i].values[0].split(' ')
                    joints = []
                    user_id = user_line[1]
                    del user_line[0]
                    del user_line[0]
                    for j in range(0, len(user_line), 6):
                        joints.append(Joint(user_line[0 + j],
                                            user_line[1 + j],
                                            user_line[2 + j],
                                            user_line[3 + j],
                                            user_line[4 + j],
                                            user_line[5 + j]))
                    person_temp = Person(user_id, joints)
                    frame_temp.persons.append(person_temp)
                self.frames.append(frame_temp)
        return self.frames


class Frame:
    def __init__(self, frame_id, number_of_persons):
        self.frame_id = frame_id
        self.number_of_persons = number_of_persons
        self.persons = []
        self.rgb_frame = None
        self.depth_frame = None

    def get_number_of_persons(self):
        return self.number_of_persons

    def get_persons(self):
        return self.persons

    def get_frame_id(self):
        return self.frame_id

    def set_images(self, rgb_frame, depth_frame):
        self.rgb_frame = rgb_frame
        self.depth_frame = depth_frame


class Person:
    def __init__(self, person_id, joints):
        self.person_id = person_id
        self.joints = joints

    def get_joints(self):
        return self.joints

    def get_person_id(self):
        return self.person_id


class Joint:
    def __init__(self, joint_name, y_image, x_image, x_world, y_world, z_world):
        self.joint_name = joint_name
        self.y_image = int(y_image)
        self.x_image = int(x_image)
        self.x_world = float(x_world)
        self.y_world = float(y_world)
        self.z_world = float(z_world)

    def get_joint_name(self):
        return self.joint_name

    def get_y_image(self):
        return self.y_image

    def get_x_image(self):
        return self.x_image

    def get_x_world(self):
        return self.x_world

    def get_y_world(self):
        return self.y_world

    def get_z_world(self):
        return self.z_world


# EXAMPLE
temp = FrameCreator(
    "./dataset/data_2.txt")
frames = temp.parse()
frames_size = len(frames)
