import numpy as np
import cv2
from math import *

rock_red_threshold = 75
rock_green_threshold = 75
rock_blue_threshold = 75

nugget_red_threshold_lower = 100
nugget_green_threshold_lower = 100
nugget_blue_threshold_lower = 0

nugget_red_threshold_upper = 260
nugget_green_threshold_upper = 220
nugget_blue_threshold_upper= 70

obstacle_threshold = (rock_red_threshold, rock_green_threshold, rock_blue_threshold)
nugget_lower = (nugget_red_threshold_lower, nugget_green_threshold_lower, nugget_blue_threshold_lower)
nugget_upper = (nugget_red_threshold_upper, nugget_green_threshold_upper,nugget_blue_threshold_upper)


def obstacle_thresh(img, rgb_thresh=(0, 0, 0)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:, :, 0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met

    #USE BITWISE AND ON NUMPY ARRAYS.
    above_thresh = (img[:, :, 0] <= rgb_thresh[0]) \
                   & (img[:, :, 1] <= rgb_thresh[1]) \
                   & (img[:, :, 2] <= rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select


def gold_nugget_thresh(img, rgb_thresh_lower=(0, 0, 0),rgb_thresh_upper=(255, 255, 255)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:, :, 0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met

    #USE BITWISE AND ON NUMPY ARRAYS.
    above_thresh = (rgb_thresh_lower[0] <= img[:, :, 0]) & (img[:, :, 0] <= rgb_thresh_upper[0]) \
                   & (rgb_thresh_lower[1] <= img[:, :, 1]) & (img[:, :, 1]<= rgb_thresh_upper[1]) \
                   & (rgb_thresh_lower[2] <= img[:, :, 2])& (img[:, :, 2]<= rgb_thresh_upper[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select


#
#
# road_threshed = color_thresh(warped)
# obstacle_threshed = obstacle_thresh(warped, obstacle_threshold)
# nugget_threshed = gold_nugget_thresh(warped_nugget, nugget_lower, nugget_upper)

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to apply a rotation to pixel positions
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    # Apply a rotation
    xpix_rotated = 0
    ypix_rotated = 0

    angle=radians(yaw)
    t=np.matrix([[cos(angle),-sin(angle),0,  0],
              [sin(angle),cos(angle),0,      0],
              [0,0,1,0],
              [0,0,0,1]])

    p=np.matrix([ xpix, ypix, 0, 1])
    p=p.transpose()

    transformed = np.dot(t, p)

    xpix_rotated = transformed[0,0]
    ypix_rotated = transformed[1,0]

    # Return the result
    # print("rotation angle", yaw)
    # print("orig x", xpix)
    # print("rotated x", xpix_rotated)
    # print("orig y", ypix)
    # print("rotated y", ypix_rotated)

    # plt.plot(xpix_rotated, ypix_rotated, '.')
    # plt.show()

    return xpix_rotated, ypix_rotated


# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
    # Apply a scaling and a translation
    xpix_translated = 0
    ypix_translated = 0

    x_world = np.int_(xpix_rot / scale)
    y_world = np.int_(ypix_rot / scale)

    t = np.matrix([[1, 0, 0, xpos],
                   [0, 1, 0, ypos],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])

    p = np.matrix([x_world, y_world, 0, 1])
    p = p.transpose()

    translated = np.dot(t, p)

    xpix_translated = translated[0,0]
    ypix_translated = translated[1,0]

    # plt.plot(xpix_translated, ypix_translated, '.')
    # plt.show()

    # Return the result
    return xpix_translated, ypix_translated

# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    half_square_width = 5
    source = np.float32([[14, 140], [302, 140], [200, 96], [118, 96]])
    destination = np.float32([[Rover.img.shape[1] / 2 - half_square_width, Rover.img.shape[0]],  # upper left
                              [Rover.img.shape[1] / 2 + half_square_width, Rover.img.shape[0]],  # lower left
                              [Rover.img.shape[1] / 2 + half_square_width, Rover.img.shape[0] - 2 * half_square_width],
                              # lower right
                              [Rover.img.shape[1] / 2 - half_square_width,
                               Rover.img.shape[0] - 2 * half_square_width]])  # upper right

    # 2) Apply perspective transform
    warped = perspect_transform(Rover.img, source, destination)

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    road_threshed = color_thresh(warped)
    obstacle_threshed = obstacle_thresh(warped, obstacle_threshold)
    nugget_threshed = gold_nugget_thresh(warped, nugget_lower, nugget_upper)

    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:,:,0] = obstacle_threshed * 255
    Rover.vision_image[:,:,1] = nugget_threshed * 255
    Rover.vision_image[:,:,2] = road_threshed * 255

    # 5) Convert map image pixel values to rover-centric coords
    road_xpix, road_ypix = rover_coords(road_threshed)
    obstacles_xpix, obstacles_ypix = rover_coords(obstacle_threshed)
    nugget_xpix, nugget_ypix = rover_coords(nugget_threshed)

    # 6) Convert rover-centric pixel values to world coordinates
    scale = 10
    road_x_world, road_y_world = pix_to_world(road_xpix, road_ypix, Rover.pos[0],
                                              Rover.pos[1], Rover.yaw,
                                              Rover.worldmap.shape[0], scale)

    obstacle_x_world, obstacle_y_world = pix_to_world(obstacles_xpix, obstacles_ypix, Rover.pos[0],
                                              Rover.pos[1], Rover.yaw, Rover.worldmap.shape[0], scale)
    nugget_x_world, nugget_y_world = pix_to_world(nugget_xpix, nugget_ypix, Rover.pos[0],
                                              Rover.pos[1], Rover.yaw,
                                              Rover.worldmap.shape[0], scale)

    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    if (Rover.roll < 0.5 or Rover.roll > 359.5) and (Rover.pitch < 0.5 or Rover.pitch > 359.5):
        #print("UPDATING WORLD MAP WITH ROLL, PITCH VALUES", Rover.roll, Rover.pitch)
        Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        Rover.worldmap[nugget_y_world, nugget_x_world, 1] += 1
        Rover.worldmap[road_y_world, road_x_world, 2] += 1
    else:
        print("NOT UPDATING WORLD MAP - ROLL, PITCH OUTSIDE THRESHOLD", Rover.roll, Rover.pitch)


    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    # Calculate pixel values in rover-centric coords and distance/angle to all pixels
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(road_xpix, road_ypix)
    Rover.nav_mean_angle = np.mean(Rover.nav_angles)

    # Check whether any rock detections are present in worldmap
    try:
        rock_world_pos = Rover.worldmap[:, :, 1].nonzero()
        if rock_world_pos[0].any():
            _, nugget_angles = to_polar_coords(nugget_xpix, nugget_ypix)
            Rover.nugget_mean_angle = np.mean(nugget_angles)
    except Exception as e:
        print("Exception taking mean for nuggets: ", str(e))
        pass
    return Rover




