#!/usr/bin/env python
import cv2
import rospy
import sys
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Point


class image_converter:

    def __init__(self):

        self.bridge = CvBridge()
        rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
        self.target_pixel = PointStamped()
        self.target_pixel.point = Point(0.0, 0.0, 0.0)

    def callback(self, data):
        try:
            # self.target_pixel.header.stamp = rospy.Time.now()

            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        except CvBridgeError as e:
            print(e)

        blur_image = cv2.GaussianBlur(gray_image, (3, 3), 0)

        circles = cv2.HoughCircles(blur_image, cv2.HOUGH_GRADIENT, 1.2, 30, None, 200)
        if circles is not None:

            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                cv2.circle(cv_image, (i[0], i[1]), i[2], (0, 255, 0), 2)
                cv2.circle(cv_image, (i[0], i[1]), 2, (0, 0, 255), 5)

                self.target_pixel.point.x = i[0]
                self.target_pixel.point.y = i[1]
            self.target_pixel.header.stamp = rospy.Time.now()

        cv_image_resize = cv2.resize(cv_image, (960, 540))
        cv2.imshow('hough circle', cv_image_resize)
        cv2.waitKey(3)


def main(args):
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()

    rate = rospy.Rate(20.0)
    pixel_pub = rospy.Publisher('/target_pixel', PointStamped, queue_size=2)
    while not rospy.is_shutdown():
        pixel_pub.publish(ic.target_pixel)
        rate.sleep()


if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass
