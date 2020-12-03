#!/usr/bin/env python3

import sys
import time

# numpy and scipy
import numpy as np
from edgetpu.detection.engine import DetectionEngine
from imutils.video import VideoStream
from PIL import Image
import argparse
import imutils
import time
import cv2

# Ros
import rospy
from sensor_msgs.msg import CompressedImage
import rospkg
from std_msgs.msg import Int16MultiArray


class FaceDetectorEdgeTPU:

    def __init__(self):
        # Read input parameters
        self.input_image_compressed = rospy.get_param('~input_image_compressed', "usb_cam/image_raw/compressed")
        self.output_image_compressed = rospy.get_param('~output_image', "face_image/compressed")
        self.model_path = rospy.get_param('~model_path', "model.tflite")
        self.threshold = rospy.get_param('~threshold', 0.8)
        self.rotation_angle = rospy.get_param('~rotation_angle', 0.0)
        self.labels_file = rospy.get_param('~labels_file', "")
        self.tracked_object = rospy.get_param('~tracked_object', "person")
        self.enable_labeling = False

        self.labels = {}

        # fix path if required
        if len(self.model_path) > 0:
            if "pkg://" in self.model_path:
                rp = rospkg.RosPack()
                path = rp.get_path('braccio_camai')
                self.model_path = self.model_path.replace("pkg://braccio_camai", path)
        else:
            rospy.loginfo("Invalid model path")
            return

        if len(self.labels_file) > 0:
            if "pkg://" in self.labels_file:
                rp = rospkg.RosPack()
                path = rp.get_path('braccio_camai')
                self.labels_file = self.labels_file.replace("pkg://braccio_camai", path)

            # loop over the class labels file
            for row in open(self.labels_file):
                # unpack the row and update the labels dictionary
                (classID, label) = row.strip().split(maxsplit=1)
                self.labels[int(classID)] = label.strip()
                
            self.enable_labeling = True

        # print input parameters
        rospy.loginfo("input_image_compressed: " + self.input_image_compressed)
        rospy.loginfo("output_image_compressed: " + self.output_image_compressed)
        rospy.loginfo("model_path: " + self.model_path)
        rospy.loginfo("threshold: " + str(self.threshold))
        rospy.loginfo("rotation_angle: " + str(self.rotation_angle))
        rospy.loginfo("labels_file: " + str(self.labels_file))
        rospy.loginfo("tracked_object: " + str(self.tracked_object))

        self.current_image = CompressedImage()
    
        rospy.loginfo("Loading Tensorflow model")
        self.model = DetectionEngine(self.model_path)

        self.pub_image = rospy.Publisher(self.output_image_compressed, CompressedImage, queue_size=1)
        self.pub_box = rospy.Publisher("bounding_box", Int16MultiArray, queue_size=1)

        self.subscriber = rospy.Subscriber(self.input_image_compressed,  CompressedImage, self.callback, queue_size=1)

        rospy.loginfo("detection started")

        while not rospy.is_shutdown():
            self.process_current_image()

        #rospy.spin()

    def process_current_image(self):
        # No image data received
        if len(self.current_image.data) == 0:
            return

        # skip is no subscribers request for detections
        if self.pub_box.get_num_connections() == 0 and self.pub_image.get_num_connections() == 0:
            return

        np_arr = np.fromstring(self.current_image.data, np.uint8)
        frame_ori = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        frame = self.rotate_image(frame_ori, self.rotation_angle)

        orig = frame.copy()
        frame = Image.fromarray(frame)

        # make predictions on the input frame
        results = self.model.DetectWithImage(frame, threshold=self.threshold, keep_aspect_ratio=True, relative_coord=False)

        # loop over the results
        for r in results:
            # extract the bounding box
            box = r.bounding_box.flatten().astype("int")
            (startX, startY, endX, endY) = box
            
            if self.enable_labeling:
                label = self.labels[r.label_id]
                if label != self.tracked_object:
                    continue # skip current object
                y = startY - 15 if startY - 15 > 15 else startY + 15
                text = "{}: {:.2f}%".format(label, r.score * 100)
                cv2.putText(orig, text, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # draw the bounding box and label on the image
            cv2.rectangle(orig, (startX, startY), (endX, endY), (0, 255, 0), 2)

            box_msg = Int16MultiArray()
            box_msg.data = [r.label_id, startX, startY, endX, endY]
            self.pub_box.publish(box_msg)
            break

        # skip if no subscribers are registered
        if self.pub_image.get_num_connections() == 0:
            return

        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', orig)[1]).tostring()

        # Publish image with face detections
        self.pub_image.publish(msg)

    def callback(self, ros_data):
        self.current_image = ros_data

    def rotate_image(self, image, angle):
        image_center = tuple(np.array(image.shape[1::-1]) / 2)
        rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
        result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
        return result


def main(args):
    rospy.init_node('FaceDetectorEdgeTPU')

    FaceDetectorEdgeTPU()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
