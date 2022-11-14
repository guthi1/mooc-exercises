#!/usr/bin/env python
# coding: utf-8

# In[1]:


# start by importing some things we will need
import cv2
import matplotlib
import numpy as np
from scipy.ndimage.filters import gaussian_filter
from scipy.stats import entropy, multivariate_normal
from math import floor, sqrt


# In[60]:


# Now let's define the prior function. In this case we choose
# to initialize the historgram based on a Gaussian distribution around [0,0]
def histogram_prior(belief, grid_spec, mean_0, cov_0):
    pos = np.empty(belief.shape + (2,))
    pos[:, :, 0] = grid_spec["d"]
    pos[:, :, 1] = grid_spec["phi"]
    RV = multivariate_normal(mean_0, cov_0)
    belief = RV.pdf(pos)
    return belief


# In[243]:


# Now let's define the predict function

def odometry(left_encoder_ticks, right_encoder_ticks, robot_spec, debug=False):
    
        wheel_radius = robot_spec['wheel_radius']
        wheel_baseline = robot_spec['wheel_baseline']
        encoder_resolution = robot_spec['encoder_resolution']

        rot_per_tick = 2 * np.pi / encoder_resolution
        delta_wheel_l =  left_encoder_ticks * rot_per_tick
        delta_wheel_r = right_encoder_ticks * rot_per_tick
        
        dist_wheel_l = delta_wheel_l * wheel_radius
        dist_wheel_r = delta_wheel_r * wheel_radius
        
        dist_robot_frame = (dist_wheel_l + dist_wheel_r)/2 # Robot distance travelled in robot frame [Meter]
        if debug:print(f"Robot has travelled: {dist_robot_frame} meters")
        
        delta_theta = (dist_wheel_l - dist_wheel_r)/wheel_baseline # Angle robot rotatiion
        if debug:print(f"Robot has ratated: {np.rad2deg(delta_theta)} degrees")
        
        return dist_robot_frame, delta_theta
        

def histogram_predict(belief, dt, left_encoder_ticks, right_encoder_ticks, grid_spec, robot_spec, cov_mask):
        belief_in = belief
        delta_t = dt
        
        dist_robot_frame, delta_theta = odometry(left_encoder_ticks, right_encoder_ticks, robot_spec)
        # We need the y coord in the world frame
        
        # TODO calculate v and w from ticks using kinematics. You will need  some parameters in the `robot_spec` defined above
        v = dist_robot_frame * np.sin(grid_spec["phi"]) / dt # replace this with a function that uses the encoder 
        w = delta_theta / dt # replace this with a function that uses the encoder
        
        # TODO propagate each centroid forward using the kinematic function
        d_t = grid_spec['d'] + v # replace this with something that adds the new odometry
        phi_t = grid_spec['phi'] + w # replace this with something that adds the new odometry

        p_belief = np.zeros(belief.shape)

        # Accumulate the mass for each cell as a result of the propagation step
        for i in range(belief.shape[0]):
            for j in range(belief.shape[1]):
                # If belief[i,j] there was no mass to move in the first place
                if belief[i, j] > 0:
                    # Now check that the centroid of the cell wasn't propagated out of the allowable range
                    if (
                        d_t[i, j] > grid_spec['d_max']
                        or d_t[i, j] < grid_spec['d_min']
                        or phi_t[i, j] < grid_spec['phi_min']
                        or phi_t[i, j] > grid_spec['phi_max']
                    ):
                        continue
                        
                    grid_d = grid_spec['d'][:,0]
                    grid_phi = grid_spec['phi'][0,:]                                       
                    i_new = np.digitize(d_t[i, j], grid_d) - 1
                    j_new = np.digitize(phi_t[i, j], grid_phi) - 1
                    
#                     d_grid_delta = grid_spec['d_max'] - grid_spec['d_min']
#                     p_grid_delta = grid_spec['phi_max'] - grid_spec['phi_min']

#                     i_size = grid_spec["d"].shape[0]  
#                     j_size = grid_spec['d'].shape[1]

#                     i_new = int((i_size*(d_t[i,j] - grid_spec["d_min"]) / (d_grid_delta)) // 1) - 1
#                     j_new = int((j_size*(phi_t[i,j] - grid_spec['phi_min']) / (p_grid_delta)) // 1) - 1

                    p_belief[i_new, j_new] += belief[i, j]

        # Finally we are going to add some "noise" according to the process model noise
        # This is implemented as a Gaussian blur
        s_belief = np.zeros(belief.shape)
        gaussian_filter(p_belief, cov_mask, output=s_belief, mode="constant")

        if np.sum(s_belief) == 0:
            return belief_in
        belief = s_belief / np.sum(s_belief)
        return belief


# In[244]:


# We will start by doing a little bit of processing on the segments to remove anything that is behing the robot (why would it be behind?)
# or a color not equal to yellow or white

def prepare_segments(segments):
    filtered_segments = []
    for segment in segments:

        # we don't care about RED ones for now
        if segment.color != segment.WHITE and segment.color != segment.YELLOW:
            continue
        # filter out any segments that are behind us
        if segment.points[0].x < 0 or segment.points[1].x < 0:
            continue

        filtered_segments.append(segment)
    return filtered_segments


# In[245]:


def generate_vote(segment, road_spec):
    p1 = np.array([segment.points[0].x, segment.points[0].y])
    p2 = np.array([segment.points[1].x, segment.points[1].y])
    t_hat = (p2 - p1) / np.linalg.norm(p2 - p1)
    n_hat = np.array([-t_hat[1], t_hat[0]])
    
    d1 = np.inner(n_hat, p1)
    d2 = np.inner(n_hat, p2)
    l1 = np.inner(t_hat, p1)
    l2 = np.inner(t_hat, p2)
    if l1 < 0:
        l1 = -l1
    if l2 < 0:
        l2 = -l2

    l_i = (l1 + l2) / 2
    d_i = (d1 + d2) / 2
    phi_i = np.arcsin(t_hat[1])
    if segment.color == segment.WHITE:  # right lane is white
        if p1[0] > p2[0]:  # right edge of white lane
            d_i -= road_spec['linewidth_white']
        else:  # left edge of white lane
            d_i = -d_i
            phi_i = -phi_i
        d_i -= road_spec['lanewidth'] / 2

    elif segment.color == segment.YELLOW:  # left lane is yellow
        if p2[0] > p1[0]:  # left edge of yellow lane
            d_i -= road_spec['linewidth_yellow']
            phi_i = -phi_i
        else:  # right edge of white lane
            d_i = -d_i
        d_i = road_spec['lanewidth'] / 2 - d_i

    return d_i, phi_i


# In[246]:


def generate_measurement_likelihood(segments, road_spec, grid_spec):

    # initialize measurement likelihood to all zeros
    measurement_likelihood = np.zeros(grid_spec['d'].shape)

    for segment in segments:
        d_i, phi_i = generate_vote(segment, road_spec)

        # if the vote lands outside of the histogram discard it
        if d_i > grid_spec['d_max'] or d_i < grid_spec['d_min'] or phi_i < grid_spec['phi_min'] or phi_i > grid_spec['phi_max']:
            continue

#         d_grid_delta = grid_spec['d_max'] - grid_spec['d_min']
#         p_grid_delta = grid_spec['phi_max'] - grid_spec['phi_min']
        
#         i_size = grid_spec["d"].shape[0]  
#         j_size = grid_spec['d'].shape[1]
        
#         i = int((i_size*(d_i - grid_spec["d_min"]) / (d_grid_delta)) // 1) - 1
#         j = int((j_size*(phi_i - grid_spec['phi_min']) / (p_grid_delta)) // 1) - 1
        
        grid_d = grid_spec['d'][:,0]
        grid_phi = grid_spec['phi'][0,:]                                       
        i = np.digitize(d_i, grid_d) - 1
        j = np.digitize(phi_i, grid_phi) - 1

        # Add one vote to that cell
        measurement_likelihood[i, j] += 1

    if np.linalg.norm(measurement_likelihood) == 0:
        return None
    measurement_likelihood /= np.sum(measurement_likelihood)
    return measurement_likelihood


# In[247]:


def histogram_update(belief, segments, road_spec, grid_spec):
    # prepare the segments for each belief array
    segmentsArray = prepare_segments(segments)
    # generate all belief arrays

    measurement_likelihood = generate_measurement_likelihood(segmentsArray, road_spec, grid_spec)

    if measurement_likelihood is not None:
        # TODO: combine the prior belief and the measurement likelihood to get the posterior belief
        # Don't forget that you may need to normalize to ensure that the output is valid probability distribution

        estimate = measurement_likelihood * belief # replace this with something that combines the belief and the measurement_likelihood
        
        # Sometime the belief goes to zero, the restart the estimation: current belief + measurement 
        if np.sum(estimate) == 0:
            estimate = belief + measurement_likelihood
    
        belief = estimate / np.sum(estimate)
    
    return (measurement_likelihood, belief)

