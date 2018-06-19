#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import datetime
import threading
#from roah_rsbb_comm_ros.msg import BenchmarkState

DISPLAY = True

class image_cropper:

    def __init__(self):
        # self.image_pub = rospy.Publisher("image_topic_2",Image)
        # Crop from top to bottom pixels (i.e 100 to 100 would take 200px off total)

        self.top = 100
        self.bottom = 40

        self.h_margin = 50
        self.v_margin = 50

        self.min_area = 1000
        self.max_area = 50000

        # Threshold values:
        self.thresh = 225
        self.maxValue = 255
        # How much to erode after thresh
        self.ed_mask = 10

        self.fgbg_flag = 0
        #self.fgbg = cv2.createBackgroundSubtractorMOG2()
        self.history = 10

        #self.frame_saver_counter = 0
        self.saver_dir = rospy.get_param("cropped_folder_path")
        rospy.loginfo("Saving to " + self.saver_dir)

        self.pub = rospy.Publisher("/camera/rgb/image_raw/crop", String, queue_size=10)
        self.pubGoal = rospy.Publisher('/hearts/or/item', String, queue_size=1)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/xtion/rgb/image_raw", Image, self.image_callback) # camera/rgb/image_raw

        #self.bench_sub = rospy.Subscriber('/roah_rsbb/benchmark/state', BenchmarkState, self.BenchStatusCallback)

        self.bench_sub = rospy.Subscriber('/hearts/cloudsight/status', String, self.BenchStatusCallback_2)

        self.count = 0
        t = threading.Timer(1.0, self.callback, [ True ])
        t.start()

    def image_callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            height, width = self.cv_image.shape[:2]

            #cv_image_cropped = cv_image
            # y: y + h, x: x + w
            self.cv_image_cropped = self.cv_image[self.top: height - self.bottom, 0:width]
            self.height, self.width = self.cv_image_cropped.shape[:2]
            # rospy.loginfo("received image!")

        except CvBridgeError as e:
            rospy.loginfo(e)

    def BenchStatusCallback_2(self, data):
        rospy.loginfo("ROAH_COMMS: BenchStatusCallback")
        if data.data == "try again":
            rospy.loginfo("Taking another image!")
            t = threading.Timer(1.0, self.callback, [ False ])
            t.start()

    def BenchStatusCallback(self, data):
        rospy.loginfo("ROAH_COMMS: BenchStatusCallback")
        if data.benchmark_state == BenchmarkState.EXECUTE:
            rospy.loginfo("EXECUTE")
            t = threading.Timer(5.0, self.callback, [ False ])
            t.start()

    def get_largest_contour_dims(self, edges, size):
        
        # find contours in edge image

        (cnts, _) = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) #cv2.CHAIN_APPROX_SIMPLE)
        cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:10]

        rospy.loginfo("PROCESSING IMAGE - n contours: " + str(len(cnts)))

        img_h = size[0]
        img_w = size[1]

        rospy.loginfo("img_h: " + str(img_h) + ", img_w: " + str(img_w))
            
        # get largest 'valid' contour e.g. by area, (x1,y1) and (x2,y2) coordinates

        for cnt in cnts:

            # get the contour bounding box 
            x1,y1,w,h = cv2.boundingRect(cnt)

            # calculate the contour area
            a = h*w
                
            # calculate bounding box (x2,y2) coordinates
            x2 = x1+w
            y2 = y1+h

            if x1 > self.h_margin and x1 < img_w - self.h_margin and \
               y1 > self.v_margin and y1 < img_h - self.v_margin and \
               x2 > self.h_margin and x2 < img_w - self.h_margin and \
               y2 > self.v_margin and y2 < img_h - self.v_margin and \
               a > self.min_area and a < self.max_area:
   

                
                # contour is 'valid', expand bounding box by v_ and h_margin
                rospy.loginfo("FOUND CONTOUR")

                x_exp = x1 - self.h_margin
                y_exp = y1 - self.v_margin
                w_exp = w + 2*self.h_margin
                h_exp = h + 2*self.v_margin
                   
                return [ x_exp, y_exp, w_exp, h_exp ]

        return None

    def callback(self, flag):
        #rospy.loginfo("RECEIVED IMAGE")
        crop_img = None
        send_raw_imgs = 0
        # Take first frame (ever) as background
        if flag:
            # self.fgmask = self.fgbg.apply(cv_image_cropped)
            rospy.loginfo("used image!")
            self.refFrame = self.cv_image_cropped
            self.fgbg_flag = self.fgbg_flag + 1
            rospy.loginfo("Got reference image")
        else:
            rospy.loginfo("Processing frame")
            #cv2.imshow("cv image", cv_image_cropped)

            # Take away reference frame from current frame
            img3 = 255 - cv2.absdiff(self.refFrame, self.cv_image_cropped)

            # Convert to gray
            gray = cv2.cvtColor(img3, cv2.COLOR_BGR2GRAY)

            #equalize
            #gray = cv2.equalizeHist(gray)
            #res = np.hstack((img,equ)) #stacking images side-by-side

            # threshold (with values above)
            th, mask = cv2.threshold(gray, self.thresh, self.maxValue, cv2.THRESH_BINARY_INV);
            # erode with values above

            
            mask = cv2.dilate(mask, np.ones((self.ed_mask, self.ed_mask)))
            mask = cv2.erode(mask, np.ones((self.ed_mask, self.ed_mask)))
            mask = cv2.dilate(mask, np.ones((self.ed_mask, self.ed_mask)))
            mask = cv2.dilate(mask, np.ones((self.ed_mask, self.ed_mask)))
            
            #mask = cv2.dilate(mask, np.ones((self.ed_mask, self.ed_mask)))
            #mask = cv2.erode(mask, np.ones((self.ed_mask, self.ed_mask)))
            
            # Use the created mask to crop the image
            res = cv2.bitwise_and(self.cv_image_cropped,self.cv_image_cropped,mask = mask)
            res[np.where((res==[0,0,0]).all(axis=2))] = [255,255,255]

            # replace black pixels with white
            res[np.where((res==[0,0,0]).all(axis=2))] = [255,255,255]

            # get object bounding box
            gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
            gray = cv2.medianBlur(gray, 3) #cv2.bilateralFilter(gray, 11, 17, 17)
            #cv2.imshow("Filtered", gray)
            
            # threshold difference image
            ret,edged = cv2.threshold(gray,127,255,0)

            # apply canny edge detector to threshold image
            edged = cv2.Canny(edged, 30, 200)
            #cv2.imshow("Edges", edged)

            dims = self.get_largest_contour_dims(edged.copy(), res.shape)

            if dims is None:
                send_raw_imgs = 1
            elif len(dims) == 4 and dims[0] >= 0 and dims[1] >= 0 and dims[2] >= 0 and dims[3] >= 0:
                rospy.loginfo("x = " + str(dims[0]) + ", y = " + str(dims[1]) + ", w = " + str(dims[2]) + ", h = " + str(dims[3]))
                crop_img = self.cv_image_cropped[dims[1]:dims[1]+dims[3], dims[0]:dims[0]+dims[2]]
                rospy.loginfo("CROPPED IMAGE")

        cv2.waitKey(3)

        if not crop_img is None:
            rospy.loginfo("Saving Images (Cropped, Blob, Raw)")

            # save raw image
            raw_file_path = self.saver_dir + "raw_img_" + str(self.count)  + ".jpg"
            cv2.imwrite(raw_file_path, self.cv_image)
            #rospy.loginfo("saved raw image")

            # save crop image
            crop_file_path = self.saver_dir + "crop_img_" + str(self.count)  + ".jpg"
            cv2.imwrite(crop_file_path, crop_img)
            #rospy.loginfo("saved crop image")

            # save blob image
            blob_file_path = self.saver_dir + "blob_img_" + str(self.count) + ".jpg"
            cv2.imwrite(blob_file_path, res)
            #rospy.loginfo("saved blob image")

            # publish crop and blob images
            msg = String()
            msg.data = ','.join([crop_file_path, blob_file_path])
            self.pub.publish(msg)
            rospy.loginfo("Sending File Paths")

            self.count = self.count + 1
            #self.frame_saver_counter = 0
        elif send_raw_imgs == 1:
            rospy.loginfo("Saving Images (Blob, Raw)")
            # save raw image
            raw_file_path = self.saver_dir + "raw_img_" + str(self.count)  + ".jpg"
            cv2.imwrite(raw_file_path, self.cv_image)
            #rospy.loginfo("saved raw image")

            # save blob image
            blob_file_path = self.saver_dir + "blob_img_" + str(self.count) + ".jpg"
            cv2.imwrite(blob_file_path, res)
            #rospy.loginfo("saved blob image")

            # publish crop and blob images
            msg = String()
            msg.data = ','.join([blob_file_path, raw_file_path])
            self.pub.publish(msg)
            rospy.loginfo("Sending file paths")

            self.count = self.count + 1
            #self.frame_saver_counter = 0
        #self.frame_saver_counter = self.frame_saver_counter + 1
        
    # Blob detection - didn't work in practise
    def mrblobby(self, img):

        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Set up the detector with default parameters.
        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()

        # Change thresholds
        params.minThreshold = 10;
        params.maxThreshold = 200;

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 150

        # Filter by Circularity
        params.filterByCircularity = False
        params.minCircularity = 0.1

        # Filter by Convexity
        params.filterByConvexity = False
        params.minConvexity = 0.87

        # Filter by Inertia
        params.filterByInertia = False
        params.minInertiaRatio = 0.01

        # Create the blobber
        detector = cv2.SimpleBlobDetector_create(params)

        keypoints = detector.detect(img)

        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(img, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        return im_with_keypoints

    # Background subtracion HOG2
    #def bgSub(self, img, ref_frame=False):

        # self.fgmask = self.fgbg.apply(img)

        #tmp_img = self.fgbg.apply(img, learningRate=1.0/self.history)

        #if(DISPLAY):
        #    cv2.imshow('fgmask',img)
        #    cv2.imshow('frame',tmp_img)


def main(args):
  rospy.init_node('image_cropper', anonymous=True)
  ic = image_cropper()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    rospy.loginfo("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
