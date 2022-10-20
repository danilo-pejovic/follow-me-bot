#!/usr/bin/env python3



import sys


import rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from pathlib import Path
import depthai as dai
import math
import cv2
import time
import argparse



BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82


TURTLEBOT3_MODEL = 'waffle_pi'

msg = """
Linear and angular velocity controller
"""

e = """
Communications Failed
"""

# List of possible objects neural network is able to detect, for our code we only need "person"
labelMap = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
            "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]

# Path to Neural Network blob
nnPathDefault = str((Path(__file__).parent /
                    Path('/home/carma/ros2_follow_ws/mobilenet-ssd_openvino_2021.4_6shave.blob')).resolve().absolute())
parser = argparse.ArgumentParser()
parser.add_argument('nnPath', nargs='?',
                    help="Path to mobilenet detection network blob", default=nnPathDefault)
parser.add_argument('-ff', '--full_frame', action="store_true",
                    help="Perform tracking on full RGB frame", default=False)

args = parser.parse_args()

fullFrameTracking = args.full_frame


def create_pipeline():
    # Create pipeline
    pipeline = dai.Pipeline()

    # Define sources and outputs, object created this way are called nodes
    camRgb = pipeline.create(dai.node.ColorCamera)
    spatialDetectionNetwork = pipeline.create(
        dai.node.MobileNetSpatialDetectionNetwork)
    monoLeft = pipeline.create(dai.node.MonoCamera)
    monoRight = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)
    objectTracker = pipeline.create(dai.node.ObjectTracker)
    xoutRgb = pipeline.create(dai.node.XLinkOut)
    trackerOut = pipeline.create(dai.node.XLinkOut)

    xoutRgb.setStreamName("preview")
    trackerOut.setStreamName("tracklets")

    # Properties
    # https://github.com/luxonis/depthai/tree/main/resources/nn holds some basic information about each NN
    # like what screen size they expect. Wrong input will lead to a crash
    camRgb.setPreviewSize(300, 300)
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camRgb.setInterleaved(False)
    camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

    monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
    monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

    # setting node configs
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    # Align depth map to the perspective of RGB camera, on which inference is done
    stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
    stereo.setOutputSize(monoLeft.getResolutionWidth(),
                        monoLeft.getResolutionHeight())

    spatialDetectionNetwork.setBlobPath(args.nnPath)
    spatialDetectionNetwork.setConfidenceThreshold(0.5)
    spatialDetectionNetwork.input.setBlocking(False)
    spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
    spatialDetectionNetwork.setDepthLowerThreshold(100)
    spatialDetectionNetwork.setDepthUpperThreshold(5000)

    objectTracker.setDetectionLabelsToTrack([15])  # track only person
    # possible tracking types: ZERO_TERM_COLOR_HISTOGRAM, ZERO_TERM_IMAGELESS, SHORT_TERM_IMAGELESS, SHORT_TERM_KCF
    objectTracker.setTrackerType(dai.TrackerType.ZERO_TERM_COLOR_HISTOGRAM)
    # take the smallest ID when new object is tracked, possible options: SMALLEST_ID, UNIQUE_ID
    objectTracker.setTrackerIdAssignmentPolicy(
        dai.TrackerIdAssignmentPolicy.SMALLEST_ID)

    # Link output of one function to be input of the other function
    # That was 2 mono cameras become a stereo camera
    # and spatial network gets feed from which it determines where a person is
    monoLeft.out.link(stereo.left)
    monoRight.out.link(stereo.right)

    camRgb.preview.link(spatialDetectionNetwork.input)
    objectTracker.passthroughTrackerFrame.link(xoutRgb.input)
    objectTracker.out.link(trackerOut.input)

    if fullFrameTracking:
        camRgb.setPreviewKeepAspectRatio(False)
        camRgb.video.link(objectTracker.inputTrackerFrame)
        objectTracker.inputTrackerFrame.setBlocking(False)
        # do not block the pipeline if it's too slow on full frame
        objectTracker.inputTrackerFrame.setQueueSize(2)
    else:
        spatialDetectionNetwork.passthrough.link(objectTracker.inputTrackerFrame)

    spatialDetectionNetwork.passthrough.link(objectTracker.inputDetectionFrame)
    spatialDetectionNetwork.out.link(objectTracker.inputDetections)
    stereo.depth.link(spatialDetectionNetwork.inputDepth)
    return pipeline


# Determine linear velocity of an robot based on proximity to the target
def linear_velocity(distance):
    sign= lambda x: math.copysign(1,x)
    if abs(distance)>100000.0:
        output=sign(distance)*WAFFLE_MAX_LIN_VEL
    elif abs(distance) > 5000.0 and abs(distance) <= 10000.0:
        output= sign(distance)*WAFFLE_MAX_LIN_VEL/2
    elif abs(distance) > 2000.0 and abs(distance) <= 5000.0:
        output = sign(distance)*WAFFLE_MAX_LIN_VEL/4
    elif abs(distance) > 1000.0 and distance <= 2000.0:
        output = sign(distance)*WAFFLE_MAX_LIN_VEL/8
    elif abs(distance) < 1000.0:
        output = 0.0
    return output

def angular_velocity(distance):
    sign= lambda x: math.copysign(1,x)
    if abs(distance)>3000.0:
        output=sign(distance)*WAFFLE_MAX_ANG_VEL
    elif abs(distance) > 1500.0 and abs(distance) <= 3000.0:
        output= sign(distance)*WAFFLE_MAX_ANG_VEL/2
    elif abs(distance) > 1000.0 and abs(distance) <= 1500.0:
        output = sign(distance)*WAFFLE_MAX_ANG_VEL/4
    elif abs(distance) > 500.0 and abs(distance) <= 1000.0:
        output = sign(distance)*WAFFLE_MAX_ANG_VEL/8
    elif abs(distance) < 500.0:
        output = 0.0
    return output



def main():
   
    with dai.Device(create_pipeline()) as device:

        # We first create a device from pipeline
        preview = device.getOutputQueue("preview", 4, False)
        tracklets = device.getOutputQueue("tracklets", 4, False)

        startTime = time.monotonic()
        counter = 0
        fps = 0
        color = (255, 255, 255)


        rclpy.init()

        qos = QoSProfile(depth=10)
        node = rclpy.create_node('bot_controller')
        pub = node.create_publisher(Twist, 'cmd_vel', qos)

        # Loop in which we get position of an target from neural network and then calculate appropriate speed and pass that information to TurtleBot.
        print(msg)
        while(1):
                imgFrame = preview.get()
                track = tracklets.get()
                

                counter += 1
                current_time = time.monotonic()
                if (current_time - startTime) > 1:
                    fps = counter / (current_time - startTime)
                    counter = 0
                    startTime = current_time

                frame = imgFrame.getCvFrame()
                trackletsData = track.tracklets
                z_value=0.0
                x_value=0.0

                for t in trackletsData:
                    roi = t.roi.denormalize(frame.shape[1], frame.shape[0])
                    x1 = int(roi.topLeft().x)
                    y1 = int(roi.topLeft().y)
                    x2 = int(roi.bottomRight().x)
                    y2 = int(roi.bottomRight().y)

                    try:
                        label = labelMap[t.label]
                    except:
                        label = t.label
                    

                    cv2.putText(frame, str(label), (x1 + 10, y1 + 20),
                                cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                    cv2.putText(
                        frame, f"ID: {[t.id]}", (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                    cv2.putText(frame, t.status.name, (x1 + 10, y1 + 50),
                                cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                    cv2.rectangle(frame, (x1, y1), (x2, y2),
                                color, cv2.FONT_HERSHEY_SIMPLEX)

                    cv2.putText(frame, f"X: {int(t.spatialCoordinates.x)} mm",
                                (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                    cv2.putText(frame, f"Y: {int(t.spatialCoordinates.y)} mm",
                                (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                    cv2.putText(frame, f"Z: {int(t.spatialCoordinates.z)} mm",
                                (x1 + 10, y1 + 95), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                    z_value=float(t.spatialCoordinates.z)

                    x_value=float(t.spatialCoordinates.x)


                cv2.putText(frame, "NN fps: {:.2f}".format(fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color)
                cv2.imshow("tracker", frame)

                if cv2.waitKey(1) == ord('q'):
                    break

                twist = Twist()
                linear_speed=linear_velocity(z_value)
                print(f'Linear distance to a person is {z_value} and linear velocity is {linear_speed}')


                twist.linear.x = linear_speed
                twist.linear.y = 0.0
                twist.linear.z = 0.0

                angular_speed= angular_velocity(x_value)
                print(f'Angular distance to a person is {x_value} and angluar velocity is {angular_speed}')

                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = angular_speed

                pub.publish(twist)




if __name__ == '__main__':
    main()
