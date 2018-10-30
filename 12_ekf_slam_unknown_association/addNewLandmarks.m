#this function adds new landmarks to the current state and current sigma
#inputs:
#  mu: mean, 
#  sigma: covariance of the robot-landmark set (x,y, theta, l_1, ..., l_N)
#
#  measurements:
#            a structure containing n measurements of landmarks
#            for each measurement we have
#            - the index of the landmark seen
#            - the location where we have seen the landmark (x,y) w.r.t the robot
#
#  id_to_state_map:
#            mapping that given the id of the measurement, returns its position in the mu vector
#  state_to_id_map:
#            mapping that given the index of mu vector, returns the id of the respective landmark
#
#outputs:
#  [mu, sigma]: the updated mean and covariance with new landmarks
#  [id_to_state_map, state_to_id_map]: the updated mapping vector between landmark position in mu vector and its id

function [mu, sigma, id_to_state_map, state_to_id_map, last_landmark_id] = addNewLandmarks(mu, sigma, measurements, id_to_state_map, state_to_id_map, last_landmark_id)

  #robot
  mu_t     = mu(1:2,:); #translational part of the robot pose
  mu_theta = mu(3);     #rotation of the robot
  c        = cos(mu_theta);
  s        = sin(mu_theta);
  R        = [c -s; s c];

  #landmarks
  M = length(measurements.observation); #number of measured landmarks
  n = (rows(mu)-3)/2;                   #current number of landmarks in the state

  #Now its time to add, if observed, the NEW landmaks, without applying any correction
  for i=1:M

    #retrieve info about the observed landmark
    measurement = measurements.observation(i);

    #if the measurement.id == -1 it means that the data association step failed for this measurement
    if (measurement.id == 0)

      #we could not associate the measurement to a landmark so we cannot use the measurement
	    continue;

    elseif(measurement.id == -1)

      #we have found a new landmark
	    measurement.id = last_landmark_id++;
    endif

    #fetch the position in the state vector corresponding to the actual measurement
    state_pos_of_landmark = id_to_state_map(measurement.id);

    #IF current landmark is a NEW landmark
    if(state_pos_of_landmark == -1) 

      #adjust direct and reverse id mappings
      n++;
      id_to_state_map(measurement.id) = n;
      state_to_id_map(n)              = measurement.id;
      
      #compute landmark position in the world
      landmark_position_in_robot = [measurement.x_pose;measurement.y_pose];
      landmark_position_in_world = mu_t + R*landmark_position_in_robot;

      #retrieve from the index the position of the landmark block in the state
      id_state = 4+2*(n-1);
 
      #adding the landmark state to the full state
      mu(id_state:id_state+1,1) = landmark_position_in_world;

      #initial noise assigned to a new landmark
      #for simplicity we put a high value only in the diagonal.
      #A more deeper analysis on the initial noise should be made.
      initial_landmark_noise = 2;
      sigma_landmark         = eye(2)*initial_landmark_noise;

      #adding the landmark covariance to the full covariance
      sigma(id_state,:)   = 0;
      sigma(id_state+1,:) = 0;
      sigma(:,id_state)   = 0;
      sigma(:,id_state+1) = 0;

      #set the covariance block
      sigma(id_state:id_state+1, id_state:id_state+1) = sigma_landmark;

      printf("observed new landmark with identifier: %i \n", measurement.id);
      fflush(stdout);
    endif
  endfor
endfunction

