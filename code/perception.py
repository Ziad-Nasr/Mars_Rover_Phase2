import numpy as np
import cv2

# Turn to true for debuggin mode, shows the pipeline on the left of the simulator's screen
DEBUGING_MODE = True

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

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
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

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
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

# a function that scales an image down in size by a factor of scale
def scale_img(img, scale):
    res = cv2.resize(img, dsize=(int(img.shape[1]/scale), int(img.shape[0]/scale)), interpolation=cv2.INTER_CUBIC)
    return res

# a function that plots the xpix and ypix of the rover coordinates in an image and shows their mean dir
def x_y_to_img(xpix, ypix, mean_dir):
    # create a blank image
    blank=np.zeros([321,161,3])
    # loop over the x and y values
    for idx, x in np.ndenumerate(xpix):
        # turn the x and y values of rover coordinates to valid image coordinates
        x = int(160 - ypix[idx])
        y = int(xpix[idx])
        # color the x and y in the image white
        blank[x,y,:] = 255
    # turn the mean angle to a vector and scale it to be seen
    x_rov = 50 * np.cos(mean_dir)
    y_rov = 50 * np.sin(mean_dir)
    # turn the mean vector from rover coordinates to image coordinates
    x = int(160 - y_rov)
    y = int(x_rov)
    # plot a square cetnered around the mean vector so the direction can be seen visually
    for i in range(5):
        for j in range(5):
            blank[np.clip(x + i, 0, 320), np.clip(y + j, 0,160), :] = [255, 0,0]
            blank[np.clip(x + i, 0, 320), np.clip(y + j, 0,160), :] = [255, 0,0]
            blank[np.clip(x - i, 0, 320), np.clip(y + j, 0,160), :] = [255, 0,0]
            blank[np.clip(x - i, 0, 320), np.clip(y + j, 0,160), :] = [255, 0,0]
    return blank

# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    image = Rover.img
    # Remove Part of the warped image to ignore the sky
    dst_size = 5
    bottom_offset = 6 
    src = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    dest = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset], 
                  [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                  ])
    # 2) Apply perspective transform
    warped = perspect_transform(image, src, dest)
    # remove top portion of the image to ignore the sky
    warped[0:100, :, :] = np.zeros_like(warped[0:100, :, :].shape)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    # if pitch is too high ignore image by turning the threshhold up to white
    thresh = (137,175,134)
    ignored_img = False
    if Rover.pitch > 1:
        ignored_img = True
        thresh = (255,255,255)
    threshold_img = color_thresh(warped, thresh)
    threshhold_obs = 1 - threshold_img
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    # moved to the bottom for debugging mode

    # 5) Convert map image pixel values to rover-centric coords
    xpix, ypix= rover_coords(threshold_img)
    rov_obs_x, rov_obs_y= rover_coords(threshhold_obs)
    # 6) Convert rover-centric pixel values to world coordinates
    xpix_world, ypix_world = pix_to_world(xpix, ypix, Rover.pos[0], Rover.pos[1], Rover.yaw,Rover.worldmap.shape[0], dst_size*2)
    obs_x, obs_y = pix_to_world(rov_obs_x, rov_obs_y, Rover.pos[0], Rover.pos[1], Rover.yaw,Rover.worldmap.shape[0], dst_size*2)

    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    # give more weight to descisions at low roll
    step = 10
    if Rover.roll > 1:
        step = 1
    Rover.worldmap[ypix_world, xpix_world, 2] += step
    Rover.worldmap[obs_y, obs_x, 0] += step
    # 8) Convert rover-centric pixel positions to polar coordinates
    dist, angles = to_polar_coords(xpix, ypix)
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    Rover.nav_dists = dist
    Rover.nav_angles = angles
    #if image was intentionally ignored maintain previous angles to keep robot going as it was
    if ignored_img:
        Rover.nav_angles = Rover.prev_angles
    # if debugging mode is on split the image on the left of the screen to multiple other images
    if DEBUGING_MODE:
        # maximum indices in the image on the left
        max_x = Rover.vision_image.shape[1] 
        max_y = Rover.vision_image.shape[0]
        # the points where we split the images both horizontally and vertically
        half_x = int(max_x/2)
        half_y = int(max_y-80)

        # show the Warped Image on the top left
        Rover.vision_image[0:half_y,0:half_x, 2] = scale_img(warped[:,:,2], 2)
        Rover.vision_image[0:half_y,0:half_x, 1] = scale_img(warped[:,:,1], 2)
        Rover.vision_image[0:half_y,0:half_x, 0] = scale_img(warped[:,:,0], 2)

        # show thes Threshholded image on the bottom left
        Rover.vision_image[half_y:max_y,0:half_x, 2] = scale_img(threshold_img * 255, 2)
        Rover.vision_image[half_y:max_y,0:half_x, 0] = scale_img(threshhold_obs * 255, 2)

        # show Rover Coordinates Image and the mean angle on the rightss
        rover_img = scale_img(x_y_to_img(xpix, ypix, np.mean(Rover.nav_angles)), 2)
        Rover.vision_image[:,half_x:half_x+80, 2] = rover_img[:,:,2]
        Rover.vision_image[:,half_x:half_x+80, 1] = rover_img[:,:,1] 
        Rover.vision_image[:,half_x:half_x+80, 0] = rover_img[:,:,0] 
    else:
        Rover.vision_image[:,:, 2] = threshold_img * 255
        Rover.vision_image[:,:, 0] = threshhold_obs * 255

    return Rover
