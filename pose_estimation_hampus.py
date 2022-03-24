from skiros2_skill.core.skill import SkillDescription, SkillBase, Sequential, ParamOptions
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.world_element import Element
import skiros2_common.tools.logger as log
import moveit_commander
import sys
import threading
import math

import rospy
from cv_bridge import CvBridge, CvBridgeError
import tf
import cv2
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import Image
from wsg_50_common.srv import Move

#from inference import Inference


class DepthListener:
    def __init__(self):
        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)
        rospy.Subscriber('/realsense/aligned_depth_to_color/image_raw', Image, callback=self.depth_callback)
        self.get = False
        self.image = None

    def depth_callback(self, data):
        if not self.get:
            return

        try:
            self.image = self.bridge.imgmsg_to_cv2(data, 'passthrough')
            self.get = False
        except CvBridgeError as e:
            return

    def get_depth(self, x, y):
        depth = 0
        while depth == 0:
            self.get = True
            while self.get:
                self.rate.sleep()
            depth = self.image[int(y * self.image.shape[0]), int(x * self.image.shape[1])]
        return depth


class RgbListener:
    def __init__(self, hsv_min, hsv_max):
        self.hsv_min = hsv_min
        self.hsv_max = hsv_max
        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)
        rospy.Subscriber('/realsense/rgb/image_raw', Image, callback=self.image_callback)
        self.get = False
        self.pos = None

    def image_callback(self, data):
        if not self.get:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)
            self.size = None
            return

        w = data.width
        h = data.height

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Thresholds for the blue block's color

        mask = cv2.inRange(hsv, self.hsv_min, self.hsv_max)

        m = cv2.moments(mask, False)
        try:
            self.pos = (m['m10']/(m['m00'] * w), m['m01']/(m['m00'] * h))
            self.get = False
            #cv2.circle(cv_image, (int(self.pos[0] * w), int(self.pos[1] * h)), 3, (0, 0, 255), -1)
        except:
            self.pos = None
            pass

        #cv2.imshow('test', cv_image)
        #cv2.waitKey(3)

    def get_block(self):
        self.get = True
        while self.get:
            self.rate.sleep()
        return self.pos


class pose_estimation_hampus(PrimitiveBase):
    def createDescription(self):
        #self.setDescription(PickWithVision(), self.__class__.__name__)
        pass

    def modifyDescription(self, skill):
        pass

    # only get depth when we have rbg, assumes publish order rpb -> depth
    def depth_callback(self, data):
        #if not self.get:
        #    return

        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, 'passthrough')
            #self.get = False
        except CvBridgeError as e:
            return

    def mid_depth(self, detection):
        box = detection['box']
        tlx = box[0].cpu()
        tly = box[1].cpu()
        brx = box[2].cpu()
        bry = box[3].cpu()

        bbox = [tlx, tly, brx, bry]

        mid_x = (tlx + brx)/2
        mid_y = (tly + bry)/2


        depth = self.depth_image[mid_x, mid_y]

        # temp stupid estimate of x and y
        x_pix = mid_x - self.image.shape[0]/2
        y_pix = mid_y - self.image.shape[1]/2

        return depth, bbox



    def rgb_callback(self, data):
        # request depth while we process rgb
        #self.get = True
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)
            self.size = None
            # abort depth request if rgb failed
            #self.get = False
            return

        predictions = self.inference.process_scene(cv_image)

        # only proceed if we have a depth image
        if self.depth_image == None:
            return

        temp_detections = []
        for pred in predictions:
            # right now depth is not compensated for size of object, TODO convert to object center
            depth, bbox = mid_depth(pred)

            # TODO convert depth and bbox to translation vector correctly, merge with rot

            ret = {'depth': depth, 'bbox': bbox, 'rot': pred['rot']}
            print(ret)





    def run(self):
        pass

    def onInit(self):
        #self.done = False
        #self.thread = threading.Thread(target=self.run)
        #self.thread.start()
        rospy.init_node('listener') #, anonymous=True)
        self.inference = Inference()
        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None
        self.get = False
        self.latest = None
        rospy.Subscriber('/realsense/rgb/image_raw', Image, callback=self.rgb_callback)
        rospy.Subscriber('/realsense/aligned_depth_to_color/image_raw', Image, callback=self.depth_callback)
        #rospy.Subscriber('/realsense/depth/points', , )
        rospy.spin() # is this needed?
        return True

    def onStart(self):
        # TODO change init and start so that onStart starts the thread if it's not running
        pass

    def onPreempt(self):
        return self.fail('Canceled', -1)

    def execute(self):
        if not self.done:
            print(self.latest)
            return self.step('Running...')

        self.thread.join()

        if self.result:
            return self.success('Done')
        else:
            return self.fail('Failed', -1)
