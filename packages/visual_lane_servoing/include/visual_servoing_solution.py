from typing import Tuple

import numpy as np
import cv2


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
    x = np.linspace(-0.1, 0, w)
    y = np.linspace(0.1, 0, h)

    xv, yv = np.meshgrid(x, y)
    steer_matrix_left = xv * yv
    scale_factor = 0.0008
    # ---
    return steer_matrix_left * scale_factor


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
    x = np.linspace(0, 0.1, w)
    y = np.linspace(0.1, 0, h)

    xv, yv = np.meshgrid(x, y)
    steer_matrix_right = xv * yv
    scale_factor = 0.0008
    # ---
    return steer_matrix_right * scale_factor


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
    # The image-to-ground homography associated with this image
    H = np.array([-4.137917960301845e-05, -0.00011445854191468058, -0.1595567007347241, 
                0.0008382870319844166, -4.141689222457687e-05, -0.2518201638170328, 
                -0.00023561657746150284, -0.005370140574116084, 0.9999999999999999])

    H = np.reshape(H,(3, 3))
    Hinv = np.linalg.inv(H)

    imghsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    x_max = 1.0
    num_points = img.shape[1]
    y_min = -1.1
    y_max = 1.35

    y_world = np.linspace(y_min, y_max, num_points)
    x_world = np.full_like(y_world, x_max)
    ones = np.ones_like(y_world)
    ground_points_world = np.stack((x_world, y_world, ones), axis=0)

    image_points_homogeneous = Hinv @ ground_points_world
    image_points = image_points_homogeneous[:2, :] / image_points_homogeneous[2, :]
    image_points = image_points.T



    mask_ground = np.zeros(img.shape, dtype=np.uint8) # TODO: CHANGE ME

    height, width = img.shape
    image_points[:, 0] = np.clip(image_points[:, 0], 0, width - 1)
    image_points[:, 1] = np.clip(image_points[:, 1], 0, height - 1)

    for point in image_points.astype(int):
        mask_ground[point[1]:, point[0]] = 1

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

    white_lower_hsv = np.array([0, 0, 150])        
    white_upper_hsv = np.array([180, 80, 255])   
    yellow_lower_hsv = np.array([10, 80, 40])        
    yellow_upper_hsv = np.array([70, 255, 255]) 
    mask_white = cv2.inRange(imghsv, white_lower_hsv, white_upper_hsv)
    mask_yellow = cv2.inRange(imghsv, yellow_lower_hsv, yellow_upper_hsv)

    mask_left_edge = mask_ground * mask_left * mask_mag_left * mask_sobelx_neg * mask_sobely_neg * mask_yellow
    mask_right_edge = mask_ground * mask_right * mask_mag_right * mask_sobelx_pos * mask_sobely_neg * mask_white

 



    return mask_left_edge, mask_right_edge
