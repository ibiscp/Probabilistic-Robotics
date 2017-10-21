#this function computes the update (also called correction)
#step of the filter
#inputs:
#  mu: mean, 
#  sigma: covariance of the robot (x,y.theta)
#  landmarks: a structure of landmarks, we can fetch the
#            position of a landmark given its index
#            by using the tools in the library
#  observations:
#            a structure containing n observations of landmarks
#            for each observation we have
#            - the index of the landmark seen
#            - the location where we have seen the landmark (x,y) w.r.t the robot
#outputs:
#  [mu, sigma]: the updated mean and covariance

function [mu, sigma] = correction(mu, sigma, landmarks, observations)

  % determine how many landmarks we have seen
  num_landmarks_seen = length(observations.observation);
  
  % dimension of the state in dim, in our case is fixed to 3
  state_dim = size(mu,1);	

  %if I've seen no landmarks, i do nothing
  if (num_landmarks_seen==0)
    return;
  endif
  
  # we precompute some quantities that come in handy later on
  mu_x = mu(1);
  mu_y = mu(2);
  mu_theta = mu(3);
  c=cos(mu_theta);
  s=sin(mu_theta);
  Rt=[c,s;-s c]; # transposed rotation matrix
  Rtp=[-s,c;-c,-s]; # derivative of transposed rotation matrix

  # here in one go, we go through all landmark measurements vector
  # for each landmark, we assemble
  # the "total" measurement vector, containing all stacked measures
  # the "total" prediction vector, containing all staked predictions
  # the "total" jacobian, consisting of all jacobians of predictions stacked
  # octave allows to "Add" rows to a matrix, thus we dynamically resize them
  
  for i=1:num_landmarks_seen
    %retrieve info about the observed landmark
    measurement = observations.observation(i);

    z_t(end+1,:) = measurement.x_pose; % where we see the landmark
    z_t(end+1,:) = measurement.y_pose;

    current_land = searchById(landmarks, measurement.id);
    lx = current_land.x_pose; % its absolute (true) position
    ly = current_land.y_pose;

    %where I should see that landmark
    t= [lx - mu_x; ly - mu_y];
    measure_prediction = Rt * t;

    h_t(end+1,:) = measure_prediction(1);
    h_t(end+1,:) = measure_prediction(2);

    %compute its Jacobian
    C = [-Rt Rtp * t];
    
    C_t(end+1,:) = C(1,:);
    C_t(end+1,:) = C(2,:);
  endfor

  %observation noise
  noise = 0.01;
  sigma_z = eye(2*num_landmarks_seen)*noise^2;
  
  %Kalman gain
  K = sigma * C_t' * (inv(C_t * sigma * C_t' + sigma_z));

  %update mu
  error = z_t - h_t;
  mu = mu + K * error;

  %update sigma
  sigma = (eye(state_dim) - K * C_t) * sigma;		

end
