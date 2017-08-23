#!/usr/bin/python
import numpy as np
import cv2
import time
import caffe

class PatternPerceptor:
    def __init__(self, trainedfile, model):
        print "pattern perceptor module added"
        self.net = caffe.Net(trainedfile, model, caffe.TEST)
        self.trainedfile = trainedfile
        self.transformer = caffe.io.Transformer({'data': self.net.blobs['data'].data.shape})
        self.transformer.set_transpose('data', (2, 0, 1))  # move image channels to outermost dimension
        self.transformer.set_mean('data', np.asarray([104, 117, 123]))  # subtract the dataset-mean value in each channel
        self.transformer.set_raw_scale('data', 255)  # rescale from [0, 1] to [0, 255]
        self.transformer.set_channel_swap('data', (2, 1, 0))  # swap channels from RGB to BGR

    def recognize(self, source):
        start = time.time()
        image = source
        transformed_image = self.transformer.preprocess('data', image)
        # copy the image data into the memory allocated for the net
        self.net.blobs['data'].data[...] = transformed_image
        ### perform classification
        output = self.net.forward()
        output_prob = output['prob'][0]  # the output probability vector for the first image in the batch
        predicted_class = output_prob.argmax()
        print ('predicted class is:', predicted_class)
        return predicted_class