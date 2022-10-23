#!/usr/bin/env python
from utils import detector_utils as detector_utils
import cv2
import tensorflow as tf
import datetime
import argparse
import cv_bridge
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image as IM
from std_msgs.msg import String

# convert ROS image format to OpenCV format
def image_callback(ros_image):
    print("got an image")
    global bridge
    global frame
    # convert ros_image into an opencv-compatible image
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "rgb8")
    except cv_bridge.CvBridgeError as e:
        print(e)
    # from now on, you can work exactly like the opencv
    frame = cv_image


# MINE
# global variables
bridge = cv_bridge.CvBridge()
# pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)
pub = rospy.Publisher('/pioneer/cmd_vel', Twist, queue_size=10)
#pub = rospy.Publisher('/gesture_class', String, queue_size=10)
rospy.init_node('gesture_recognition_node', anonymous=True)
rate = rospy.Rate(10)
#image_sub = rospy.Subscriber("/camera/image_raw", IM, image_callback)
#image_sub = rospy.Subscriber("/csi_cam_0/image_raw", IM, image_callback)
frame = None


#cap = cv2.VideoCapture(0)
detection_graph, sess = detector_utils.load_inference_graph()





if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-sth', '--scorethreshold', dest='score_thresh', type=float,
                        default=0.5, help='Score threshold for displaying bounding boxes')
    parser.add_argument('-fps', '--fps', dest='fps', type=int,
                        default=1, help='Show FPS on detection/display visualization')
    parser.add_argument('-src', '--source', dest='video_source',
                        default=0, help='Device index of the camera.')
    parser.add_argument('-wd', '--width', dest='width', type=int,
                        default=960, help='Width of the frames in the video stream.')
    parser.add_argument('-ht', '--height', dest='height', type=int,
                        default=640, help='Height of the frames in the video stream.')
    parser.add_argument('-ds', '--display', dest='display', type=int,
                        default=1, help='Display the detected images using OpenCV. This reduces FPS')
    parser.add_argument('-num-w', '--num-workers', dest='num_workers', type=int,
                        default=4, help='Number of workers.')
    parser.add_argument('-q-size', '--queue-size', dest='queue_size', type=int,
                        default=5, help='Size of the queue.')
    args = parser.parse_args()

    # for direct access to camera
    ''' 
    if not cap.isOpened():
        print("camera or video open failed")

    im_width, im_height = (cap.get(3), cap.get(4))
    size = (int(im_width), int(im_height))
    '''


    start_time = datetime.datetime.now()
    num_frames = 0

    # max number of hands we want to detect/track
    num_hands_detect = 1

    (im_width, im_height) = frame.shape[:2]

    try:
        while not rospy.is_shutdown():

            # for direct access to camera
            # ret, frame = cap.read()

            # actual detection  
            boxes, scores, classes = detector_utils.detect_objects(
                frame, detection_graph, sess)

            #print(classes[0]) #MINE
            #pub.publish(classes[0]) #MINE


            # print scores[0],classes[0]
            # draw bounding boxes
            gesture_class = detector_utils.draw_box_on_image(
                num_hands_detect, args.score_thresh, scores, boxes, im_width, im_height, frame, classes)

            pub.publish(gesture_class)  # MINE

            # Calculate Frames per second (FPS)
            num_frames += 1
            elapsed_time = (datetime.datetime.now() -
                            start_time).total_seconds()
            fps = num_frames / elapsed_time

            if (args.display > 0):
                # Display FPS on frame
                if (args.fps > 0):
                    detector_utils.draw_fps_on_image(
                        "FPS : " + str(int(fps)), frame)

                cv2.imshow('Single Threaded Detection', cv2.cvtColor(
                    frame, cv2.COLOR_RGB2BGR))

                if cv2.waitKey(5) & 0xFF == ord('q'):
                    cv2.destroyAllWindows()
                    break
            else:
                print("frames processed: ", num_frames,
                      "elapsed time: ", elapsed_time, "fps: ", str(int(fps)))

            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
        # keep looping, until interrupted
