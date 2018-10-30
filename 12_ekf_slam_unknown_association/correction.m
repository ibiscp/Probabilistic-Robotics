#this function computes the update (also called correction)
#step of the filter
#inputs:
#  mu: mean, 
#  sigma: covariance of the robot-landmark set (x,y, theta, l_1, ..., l_N)
#
#  observations:
#            a structure containing n observations of landmarks
#            for each observation we have
#            - the index of the landmark seen
#            - the location where we have seen the landmark (x,y) w.r.t the robot
#
#  id_to_state_map:
#            mapping that given the id of the measurement, returns its position in the mu vector
#  state_to_id_map:
#            mapping that given the index of mu vector, returns the id of the respective landmark
#
#outputs:
#  [mu, sigma]: the updated mean and covariance
#  [id_to_state_map, state_to_id_map]: the updated mapping vector between landmark position in mu vector and its id

function [mu, sigma, id_to_state_map, state_to_id_map] = correction(mu, sigma, observations, id_to_state_map, state_to_id_map)

  #determine how many landmarks we have seen in this step
  M = length(observations.observation);

  #if I've seen no landmarks, i do nothing
  if (M == 0)
    return;
  endif

  #dimension of the state (robot pose + landmark positions)
  dimension_mu = size(mu,1);
  mu_robot     = mu(1:3,:);

  # re precompute some quantities that come in handy later on
  c   = cos(mu_robot(3));
  s   = sin(mu_robot(3));
  R   = [c -s; s c];  #rotation matrix
  Rt  = [c,s;-s c];    #transposed rotation matrix
  Rtp = [-s,c;-c,-s]; #derivative of transposed rotation matrix

  # for below computation, we need to count how many observations 
  # of old landmarks we have
  number_of_known_landmarks = 0;

  # Here two cases arise, the current landmark has been already seen, i.e. REOBSERVED landmark,
  # or the current landmark is completely new, i.e. NEW landmark.
  #
  # for simplicity we can divide the problem: first analyze only the reobserved landmark
  # and work with them as in a localization procedure (of course, with full Jacobian now).
  # With this reobserved landmark we compute the correction/update of mean and covariance.
  # Then, after the correction is done, we can simply add the new landmark expanding the
  # mean and the covariance.
  #
  # First of all we are interested in REOBSERVED landmark 
  for i=1:M
  
    #retrieve info about the observed landmark
    measurement = observations.observation(i);

    #if the measurement.id < 1 it means that the data association step failed or is in doubt for this measurement
    #we cannot use it for our correction - skip it
    if (measurement.id < 1)
	    continue;
    endif

    #fetch the position in the state vector corresponding to the actual measurement
    n = id_to_state_map(measurement.id);

    #IF current landmark is a REOBSERVED LANDMARK
    if(n != -1)

      #compute the index (vector coordinate) in the state vector corresponding to the pose of the landmark;	
      id_state = 4+2*(n-1);

      #where we see the landmark
      z_t(end+1,:) = measurement.x_pose; 
      z_t(end+1,:) = measurement.y_pose;

      #fetch the position of the landmark in the state (its x and y coordinate)
      landmark_position = mu(id_state:id_state+1,:);
      
      #where I predict i will see that landmark
      [measurement_prediction, C_m] = measurement_function(dimension_mu, mu_robot, landmark_position, id_state);

      #add prediction to prediction vector
      h_t(end+1,:) = measurement_prediction(1);
      h_t(end+1,:) = measurement_prediction(2);

      #add jacobian
      C_t(end+1,:) = C_m(1,:);
      C_t(end+1,:) = C_m(2,:);

      #increment the counter of observations originating from already known landmarks
      number_of_known_landmarks++;
    endif
  endfor

  #if I have seen again at least one landmark
  #I need to update, otherwise I jump to the new landmark case
  if ((number_of_known_landmarks > 0))

    #observation noise
    noise   = 0.01;
    sigma_z = eye(2*number_of_known_landmarks)*noise;

    #Kalman gain
    K = sigma * C_t'*(inv(C_t*sigma*C_t' + sigma_z));

    #update mu
    innovation = (z_t - h_t);
    mu         = mu + K*innovation;

    #update sigma
    sigma = (eye(dimension_mu) - K*C_t)*sigma;		
  endif
end

