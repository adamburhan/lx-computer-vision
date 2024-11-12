from typing import Tuple

import numpy as np
import cv2
import rospy


def get_steer_matrix_left_lane_markings(shape: Tuple[int, int]) -> np.ndarray:
    """
    Args:
        shape:              The shape of the steer matrix.

    Return:
        steer_matrix_left:  The steering (angular rate) matrix for reactive control
                            using the masked left lane markings (numpy.ndarray)
    """

    # TODO: implement your own solution here
    h, w = shape
    x = -1 * np.ones((h, w))

    x[:(h//2), :] = 0

    scale_factor = 0.01
    steer_matrix_left = x * rospy.get_param('scale_right')
    
    # ---
    return steer_matrix_left 


def get_steer_matrix_right_lane_markings(shape: Tuple[int, int]) -> np.ndarray:
    """
    Args:
        shape:               The shape of the steer matrix.

    Return:
        steer_matrix_right:  The steering (angular rate) matrix for reactive control
                             using the masked right lane markings (numpy.ndarray)
    """

    # TODO: implement your own solution here
    h, w = shape
    x = np.ones((h, w))

    x[:(h//2), :] = 0

    scale_factor = 0.01
    steer_matrix_right = x * rospy.get_param('scale_right')
    # ---
    return steer_matrix_right


def detect_lane_markings(image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Args:
        image: An image from the robot's camera in the BGR color space (numpy.ndarray)
    Return:
        mask_left_edge:   Masked image for the dashed-yellow line (numpy.ndarray)
        mask_right_edge:  Masked image for the solid-white line (numpy.ndarray)
    """
    h, w, _ = image.shape

    # TODO: implement your own solution here

    """ 
    1. blur the image with gaussian blur

    2. apply sobel operator for edges (x and y gradients)

    3. Now we have a reduced number of candidate edges but preserved most of the lane marking edges
       Assumption: lane marking edges are stronger than most of the edges in the image (gradient magnitude)
     
    4. Now we have a set of candidate edges and want a set of masks to isolate dashed-yellow/solid white lane markings
    """
    img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    imghsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # blurring with Gaussian blur
    sigma = 3
    img_gaussian_filter = cv2.GaussianBlur(img, (0, 0), sigma)

    # sobel operator for edges
    sobelx = cv2.Sobel(img_gaussian_filter, cv2.CV_64F, 1, 0)
    sobely = cv2.Sobel(img_gaussian_filter, cv2.CV_64F, 0, 1)

    # gradient magnitudes and directions
    Gmag = np.sqrt(sobelx*sobelx + sobely*sobely)
    Gdir = cv2.phase(np.array(sobelx, np.float32), np.array(sobely, dtype=np.float32), angleInDegrees=True)

    mask_left = np.ones((h, w))
    mask_left[:, int(np.floor(w/2)):w + 1] = 0
    mask_right = np.ones((h, w))
    mask_right[:, 0:int(np.floor(w/2))] = 0

    mask_sobelx_pos = (sobelx > 0)
    mask_sobelx_neg = (sobelx < 0)
    mask_sobely_pos = (sobely > 0)
    mask_sobely_neg = (sobely < 0)

    threshold_left = 5
    threshold_right = 80
    mask_mag_left = (Gmag > threshold_left)
    mask_mag_right = (Gmag > threshold_right)

    white_lower_hsv = np.array(rospy.get_param('white_low'))        
    white_upper_hsv = np.array(rospy.get_param('white_high'))   
    yellow_lower_hsv = np.array(rospy.get_param('yellow_low'))        
    yellow_upper_hsv = np.array(rospy.get_param('yellow_high')) 
    mask_white = cv2.inRange(imghsv, white_lower_hsv, white_upper_hsv)
    mask_yellow = cv2.inRange(imghsv, yellow_lower_hsv, yellow_upper_hsv)

    mask_left_edge = mask_left * mask_mag_left * mask_sobelx_neg * mask_sobely_neg * mask_yellow
    mask_right_edge = mask_right * mask_mag_right * mask_sobelx_pos * mask_sobely_neg * mask_white

 



    return mask_left_edge, mask_right_edge
