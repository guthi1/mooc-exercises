#!/usr/bin/env python
# coding: utf-8

# In[39]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your DeltaPhi function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK

import cv2
import numpy as np


def get_steer_matrix_left_lane_markings(shape):
    """
        Args:
            shape: The shape of the steer matrix (tuple of ints)
        Return:
            steer_matrix_left_lane: The steering (angular rate) matrix for Braitenberg-like control 
                                    using the masked left lane markings (numpy.ndarray)
    """
    
    steer_matrix_left_lane = np.zeros(shape)
    half = shape[1]//2
    steer_matrix_left_lane[:, :half] = -3
    
#     val = -10.0
#     for i in range(shape[1] - half):
#         steer_matrix_left_lane[:, half - i - 1] = val
#         val -= 0.3
    
    # steer_matrix_left_lane = np.ones(shape) * -10

    return steer_matrix_left_lane

# In[41]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your DeltaPhi function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK


def get_steer_matrix_right_lane_markings(shape):
    """
        Args:
            shape: The shape of the steer matrix (tuple of ints)
        Return:
            steer_matrix_right_lane: The steering (angular rate) matrix for Braitenberg-like control 
                                     using the masked right lane markings (numpy.ndarray)
    """
    
    steer_matrix_right_lane = np.zeros(shape)
    half = shape[1]//2
    steer_matrix_right_lane[:, half:] = 1
    
    # val = 4.0
    # for i in range(shape[1] - half):
    #     steer_matrix_right_lane[:, half + i] = val
    #     val += 0.05
        
    return steer_matrix_right_lane

# In[33]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your DeltaPhi function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK

import cv2
import numpy as np


def detect_lane_markings(image):
    """
        Args:
            image: An image from the robot's camera in the BGR color space (numpy.ndarray)
        Return:
            left_masked_img:   Masked image for the dashed-yellow line (numpy.ndarray)
            right_masked_img:  Masked image for the solid-white line (numpy.ndarray)
    """
    
    h, w, _ = image.shape
    
    # Generate hsv and grayscale versions
    imgbgr = image
    # OpenCV uses BGR by default, whereas matplotlib uses RGB, so we generate an RGB version for the sake of visualization
    imgrgb = cv2.cvtColor(imgbgr, cv2.COLOR_BGR2RGB)
    # Convert the image to HSV for any color-based filtering
    imghsv = cv2.cvtColor(imgbgr, cv2.COLOR_BGR2HSV)
    # Most of our operations will be performed on the grayscale version
    img = cv2.cvtColor(imgbgr, cv2.COLOR_BGR2GRAY)
    
    # instantiate them with dtype=np.uint8
    mask_ground = np.ones(img.shape, dtype=np.uint8) # TODO: CHANGE ME
    mask_ground[:170, :] = 0

    # Smooth the image using a Gaussian kernel
    sigma = 4
    img_gaussian_filter = cv2.GaussianBlur(img, (0,0), sigma)

    # Convolve the image with the Sobel operator (filter) to compute the numerical derivatives in the x and y directions
    sobelx = cv2.Sobel(img_gaussian_filter,cv2.CV_64F,1,0)
    sobely = cv2.Sobel(img_gaussian_filter,cv2.CV_64F,0,1)

    # Compute the magnitude of the gradients and orientation of the gradients
    Gmag = np.sqrt(sobelx*sobelx + sobely*sobely)
    Gdir = cv2.phase(np.array(sobelx, np.float32), np.array(sobely, dtype=np.float32), angleInDegrees=True)

    # TODO: Use the histogram above to choose the minimum threshold on the gradient magnitude. 
    #       Edges whos gradient magnitude is below this threshold will be filtered out.
    threshold = 55 # CHANGE ME
    mask_mag = (Gmag > threshold)

    # Color-based Masking
    white_lower_hsv = np.array([0, 0, 140])      
    white_upper_hsv = np.array([150, 50, 255]) 
    yellow_lower_hsv = np.array([15, 100, 100])   
    yellow_upper_hsv = np.array([40, 255, 255])  
    mask_white = cv2.inRange(imghsv, white_lower_hsv, white_upper_hsv)
    mask_yellow = cv2.inRange(imghsv, yellow_lower_hsv, yellow_upper_hsv)

    # Edge-based Masking
    # create masks for the left- and right-halves of the image
    width = img.shape[1]
    
    mask_left = np.ones(sobelx.shape)
    mask_left[:,int(np.floor(width/2)):width + 1] = 0
    mask_right = np.ones(sobelx.shape)
    mask_right[:,0:int(np.floor(width/2))] = 0

    # In the left-half image, we are interested in the right-half of the dashed yellow line, which corresponds to negative x- and y-derivatives
    # In the right-half image, we are interested in the left-half of the solid white line, which correspons to a positive x-derivative and a negative y-derivative
    # Generate a mask that identifies pixels based on the sign of their x-derivative
    mask_sobelx_pos = (sobelx > 0)
    mask_sobelx_neg = (sobelx < 0)
    mask_sobely_pos = (sobely > 0)
    mask_sobely_neg = (sobely < 0)

    # Let's combine these masks with the gradient magnitude mask
    mask_left_edge =  mask_left * mask_mag * mask_sobelx_neg * mask_sobely_neg * mask_yellow
    mask_right_edge =  mask_right * mask_mag * mask_sobelx_pos * mask_sobely_neg * mask_white

    return (mask_left_edge, mask_right_edge)
