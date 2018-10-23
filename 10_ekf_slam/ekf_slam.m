close all
clear
clc

#load dependencies
addpath "../../"
addpath "../tools/g2o_wrapper"
addpath "../tools/visualization"
source "../tools/utilities/geometry_helpers_2d.m"

addpath "./exercise" #uncomment this line to target the exercise
#addpath "./solution" #comment this line to target the exercise

#load your own dataset dataset, without landmarks (first entry remains empty)
[_, poses, transitions, observations] = loadG2o("../datasets/dataset_point.g2o");

#set initial pose at the origin - we don't know the map and neither our location
mu = [0;  #x coordinate
      0;  #y coordinate
      0]; #orientation theta (yaw angle)
printf("initial pose: [%f, %f, %f]\n", mu(1), mu(2), mu(3));

#initialize covariance: high value means high uncertainty
sigma = eye(3);

#bookkeeping: to and from mapping between robot pose (x,y, theta) and landmark indices (i)
#all mappings are initialized with invalid value -1 (meaning that the index is not mapped)
#since we do not know how many landmarks we will observe, we allocate a large enough buffer
id_to_state_map = ones(10000, 1)*-1;
state_to_id_map = ones(10000, 1)*-1;

#------------------------------------------ VISUALIZATION ONLY ------------------------------------------
#initialize GUI with initial situation
figure("name", "ekf_slam",    #figure title
       "numbertitle", "off"); #remove figure number
trajectory = [mu(1), mu(2)];
#------------------------------------------ VISUALIZATION ONLY ------------------------------------------

#simulation cycle: for the number of transitions recorded in the dataset
for t = 1:length(transitions)

  #obtain current transition
  transition = transitions(t);
  
  #obtain current observations
  observations_t = observations(t);

  #EKF predict
  [mu, sigma] = prediction(mu, sigma, transition);

  #EKF correct
  [mu, sigma] = correction(mu, sigma, observations_t, id_to_state_map, state_to_id_map);

  #ADD new landmarks to the state
  [mu, sigma, id_to_state_map, state_to_id_map] = addNewLandmarks(mu, sigma, observations_t, id_to_state_map, state_to_id_map);

#------------------------------------------ VISUALIZATION ONLY ------------------------------------------
  #display current state - transform data
  N = (rows(mu)-3)/2;
  if N > 0
    landmarks = landmark(state_to_id_map(1), [mu(4), mu(5)]);
    for u = 2:N
      landmarks(end+1) = landmark(state_to_id_map(u), [mu(3+2*u-1), mu(3+2*u)]);
    endfor
    printf("current pose: [%f, %f, %f], map size (landmarks): %u\n", mu(1), mu(2), mu(3), N);
    trajectory = [trajectory; mu(1), mu(2)];
    plotState(landmarks, mu, sigma, observations_t, trajectory);
  endif
  pause(.1)
  fflush(stdout);	
#------------------------------------------ VISUALIZATION ONLY ------------------------------------------
endfor

