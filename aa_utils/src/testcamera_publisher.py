#!/usr/bin/env python




#rosrun python_scrips_ros camera_publisher6.py _pub_topic_l:=/left/camera_info _pub_topic_r:=/right/camera_info _file:=/home/alex/Documents/Data_Bags/MH/sensors.yaml _sub_topic_l:=/left/image_raw _sub_topic_r:=/right/image_raw


import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
import subprocess, yaml
import os.path
import numpy as np
from sensor_msgs.msg import Image
from cv2 import cv
import cv2
import Queue
import copy

class LookupTable:

    rmap=cv.CreateMat( 2,2, cv.CV_64F)
    T_disp2depth=cv.CreateMat( 4,4, cv.CV_32F)

    rectifiedFocalLen=0.0
    rectifiedCenterCol=0.0
    rectifiedCenterRow=0.0
    baseline=0.0




class MyCameras:
    camera_left = CameraInfo()
    camera_right = CameraInfo()
    calculated=False

    def publish_camera(self,pub_topic,camera):
        print "publishing camera"
        pub = rospy.Publisher(pub_topic, CameraInfo, queue_size=1)
        pub.publish(camera)

    def calculate_params(self,file_path):
        global lookupTable
        print "calculating params for the cameras"
        if not os.path.isfile(file_path) and os.access(file_path, os.R_OK):
            print "Either file is missing or is not readable"
            exit(-1)
        stream= open (file_path, "r")
        doc = yaml.load(stream)

        '''
        read from the file the width and the height and asign it to both cameras
        read the distorsion model but assign only pumb_bob because that one is requiered by sptam
        read the distCoeffs for both cameras and assign the lists to the cameras
        read k, make a list of 9 elements with the correct positions out of it and then assign t to both cameras

        Read the extrinsics of both cameras, inverse one of them and calculate the R and t from camera_left to camera_left_to_right


        make everything be a cv matrix
        cv.StereoRectify(K1_mat, K2_mat, distCoeffs1_mat, distCoeffs2_mat, imageSize, R, t_mat, R1, R2, P1, P2,T_disp2depth, flags=cv.CV_CALIB_ZERO_DISPARITY, alpha=-1, newImageSize=(0,0))
        make P1 into a list and assign it as  camera_left.P
        make P2 into a list and assign it as  camera_right.P
        make R1 into a list and assign it as  camera_left.R
        make R2 into a list and assign it as  camera_right.R
        '''


        #width and height
        self.camera_left.width=doc['camera0']['resolution'][0]
        self.camera_left.height=doc['camera0']['resolution'][1]
        self.camera_right.width=doc['camera1']['resolution'][0]
        self.camera_right.height=doc['camera1']['resolution'][1]

        #distortion_model
        self.camera_left.distortion_model="plumb_bob"
        self.camera_right.distortion_model="plumb_bob"

        #distortion_coefficients
        self.camera_left.D=doc['camera0']['distortion_coefficients']
        self.camera_right.D=doc['camera1']['distortion_coefficients']


        #K_left
        self.camera_left.K[0]=doc['camera0']['intrinsics'][0]
        self.camera_left.K[1]=0.0
        self.camera_left.K[2]=doc['camera0']['intrinsics'][2]
        self.camera_left.K[3]=0.0
        self.camera_left.K[4]=doc['camera0']['intrinsics'][1]
        self.camera_left.K[5]=doc['camera0']['intrinsics'][3]
        self.camera_left.K[6]=0.0
        self.camera_left.K[7]=0.0
        self.camera_left.K[8]=1.0

        #K_right
        self.camera_right.K[0]=doc['camera1']['intrinsics'][0]
        self.camera_right.K[1]=0.0
        self.camera_right.K[2]=doc['camera1']['intrinsics'][2]
        self.camera_right.K[3]=0.0
        self.camera_right.K[4]=doc['camera1']['intrinsics'][1]
        self.camera_right.K[5]=doc['camera1']['intrinsics'][3]
        self.camera_right.K[6]=0.0
        self.camera_right.K[7]=0.0
        self.camera_right.K[8]=1.0

        #R and t from camera left to right
        extrinsics_4x4_l = np.array( doc ['camera0']['T_BS']['data'])
        extrinsics_left=extrinsics_4x4_l.reshape(4,4)
        extrinsics_4x4_r = np.array( doc ['camera1']['T_BS']['data'])
        extrinsics_right=extrinsics_4x4_r.reshape(4,4)
        inverse=np.linalg.inv(extrinsics_left)
        camera_left_to_right=np.matrix(inverse)* np.matrix(extrinsics_right)
        camera_left_to_right=camera_left_to_right[0:3,0:4]
        R=camera_left_to_right[0:3,0:3]
        t=camera_left_to_right[0:3,3]

        stream.close()
        #Prepare values for rectifying function calling
        R1 = cv.CreateMat( 3,3, cv.CV_64F)
        R2 = cv.CreateMat(3,3, cv.CV_64F)
        P1 = cv.CreateMat( 3,4, cv.CV_64F)
        P2 = cv.CreateMat(3,4, cv.CV_64F)

        K1_arr = np.asarray(self.camera_left.K)
        K1_arr=K1_arr.reshape(3,3)
        K2_arr = np.asarray(self.camera_right.K)
        K2_arr=K2_arr.reshape(3,3)
        K1_mat_cont=  cv.fromarray(cv2.copyMakeBorder(K1_arr,0,0,0,0,cv2.BORDER_REPLICATE))
        K2_mat_cont= cv.fromarray( cv2.copyMakeBorder(K2_arr,0,0,0,0,cv2.BORDER_REPLICATE))

        distCoeffs1_array=np.matrix(np.array(self.camera_left.D))
        distCoeffs2_array=np.matrix(np.array(self.camera_right.D))
        distCoeffs1_mat_cont=  cv.fromarray(cv2.copyMakeBorder(distCoeffs1_array,0,0,0,0,cv2.BORDER_REPLICATE))
        distCoeffs2_mat_cont= cv.fromarray( cv2.copyMakeBorder(distCoeffs2_array,0,0,0,0,cv2.BORDER_REPLICATE))

        R_mat_cont=  cv.fromarray(cv2.copyMakeBorder(R,0,0,0,0,cv2.BORDER_REPLICATE))
        t_mat_cont=  cv.fromarray(cv2.copyMakeBorder(t,0,0,0,0,cv2.BORDER_REPLICATE))

        imageSize= (self.camera_left.width, self.camera_left.height)

        '''
        t[0,0]=-50.706459062
        t[1,0]=0.15556820057
        t[2,0]=0.00088938278
        '''

        T_disp2depth = cv.CreateMat( 4,4, cv.CV_32F)

        #######################

        cv.StereoRectify(K1_mat_cont, K2_mat_cont, distCoeffs1_mat_cont, distCoeffs2_mat_cont, imageSize, R_mat_cont, t_mat_cont, R1, R2, P1, P2,T_disp2depth, flags=cv.CV_CALIB_ZERO_DISPARITY, alpha=-1, newImageSize=(0,0))

        lookupTable.rectifiedFocalLen = P1[0,0]
        lookupTable.rectifiedCenterCol = P1[0, 2];
        lookupTable.rectifiedCenterRow = P1[1, 2];
        #lookupTable.baseline = cv.norm(t_mat);

        cv.InitUndistortRectifyMap(K1_mat_cont, distCoeffs1_mat_cont, R1, P1, None, None)
        cv.InitUndistortRectifyMap(K2_mat_cont, distCoeffs2_mat_cont, R2, P2, None, None)

        #Transform R1,R2 and P1 and P2 and put thhem in the cameras
        P1_arr = np.asarray( P1[:,:] )
        P1_arr=np.squeeze(np.asarray(P1_arr)).flatten()
        P2_arr = np.asarray( P2[:,:] )
        P2_arr=np.squeeze(np.asarray(P2_arr)).flatten()
        R1_arr = np.asarray( R1[:,:] )
        R1_arr=np.squeeze(np.asarray(R1_arr)).flatten()
        R2_arr = np.asarray( R2[:,:] )
        R2_arr=np.squeeze(np.asarray(R2_arr)).flatten()
        R_id_arr=[1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,1.0]

        self.camera_left.R=R_id_arr
        self.camera_right.R=np.squeeze(np.asarray(R)).flatten()
        self.camera_left.P=P1_arr
        self.camera_right.P=P2_arr

        # Tx = - fx * B
        self.camera_right.P[3]= - self.camera_right.P[3] * self.camera_right.K[0]


        print "finished calculating params, we return"
        self.calculated=True
        return

cameras= MyCameras()
lookupTable= LookupTable()

def img_cb_l(img):
    global cameras
    global lookupTable

    stamp=img.header.stamp
    pub_topic = rospy.get_param('~pub_topic_l', 'camera_info_error')

    cameras.camera_left.header.stamp=stamp
    cameras.publish_camera(pub_topic,cameras.camera_left)


def img_cb_r(img):
    global cameras
    global lookupTable

    stamp=img.header.stamp
    pub_topic = rospy.get_param('~pub_topic_r', 'camera_info_error')

    cameras.camera_right.header.stamp=stamp
    cameras.publish_camera(pub_topic,cameras.camera_right)



def talker():

    rospy.init_node('camera_publisher')

    file_path = rospy.get_param('~infile', 'invalid_yaml.yaml')
    if not cameras.calculated:
        cameras.calculate_params (file_path)
    print "now the cameras params should be calculated"


    sub_topic_left=rospy.get_param('~sub_topic_l', 'std_msgs/String')
    print "subscribing to topic" , sub_topic_left,  "of the image (used for syncronization)"
    sub_topic_right=rospy.get_param('~sub_topic_r', 'std_msgs/String')
    print "subscribing to topic" , sub_topic_right,  "of the image (used for syncronization)"

    rospy.Subscriber(sub_topic_left, Image, img_cb_l)
    rospy.Subscriber(sub_topic_right, Image, img_cb_r)






    rospy.spin()






    #from the yaml file od dpptam. It is hardocded now but it shoudl read it dynamically


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
