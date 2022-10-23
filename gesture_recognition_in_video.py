#!/usr/bin/env python
from utils import detector_utils as detector_utils
import cv2
import tensorflow as tf
import datetime
import argparse
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image as IM
from std_msgs.msg import String

cap = cv2.VideoCapture(0)
detection_graph, sess = detector_utils.load_inference_graph()
rospy.init_node('gesture_recognition_node', anonymous=True)
# pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)
pub_cmd = rospy.Publisher('/pioneer/cmd_vel', Twist, queue_size=10)
pub = rospy.Publisher('/gesture_class', String, queue_size=10)
vel_msg = Twist()
rate = rospy.Rate(10)

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('-sth', '--scorethreshold', dest='score_thresh', type=float,
                        default=0.01, help='Score threshold for displaying bounding boxes')
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


    if not cap.isOpened():
        print("camera or video open failed")
    

    im_width, im_height = (cap.get(3), cap.get(4))
    size = (int(im_width), int(im_height))



    start_time = datetime.datetime.now()
    num_frames = 0

    # max number of hands we want to detect/track
    num_hands_detect = 1

    #while True:
    try:
        while not rospy.is_shutdown():
            # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
            ret, image_np = cap.read()
            # image_np=cv2.imread("990.jpg")





            # actual detection
            boxes, scores,classes = detector_utils.detect_objects(
                image_np, detection_graph, sess)
            # TODO na valw na epistrefei h sunartisi tin kathgoria kai na kanei analogws ta publish sto topic opws eipame

            #print scores[0],classes[0]


            # draw bounding boxes
            class_name = detector_utils.draw_box_on_image(num_hands_detect, args.score_thresh, scores, boxes, im_width, im_height, image_np, classes)
            print(class_name)
            pub.publish(class_name)  # MINE
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            pub_cmd.publish(vel_msg)

            # Calculate Frames per second (FPS)
            num_frames += 5
            elapsed_time = (datetime.datetime.now() -
                            start_time).total_seconds()
            fps = num_frames / elapsed_time

            if (args.display > 0):
                # Display FPS on frame
                if (args.fps > 0):
                    detector_utils.draw_fps_on_image(
                        "FPS : " + str(int(fps)), image_np)

                cv2.imshow('Single Threaded Detection', cv2.cvtColor(
                    image_np, cv2.COLOR_RGB2BGR))

                image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)

                if cv2.waitKey(5) & 0xFF == ord('q'):
                    cv2.destroyAllWindows()
                    break
            else:
                print("frames processed: ",  num_frames,
                      "elapsed time: ", elapsed_time, "fps: ", str(int(fps)))
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
        # keep looping, until interrupted
