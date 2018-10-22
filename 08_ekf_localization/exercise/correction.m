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
  Rt=[c s;-s c]; # transposed rotation matrix
  Rtp=[-s c;-c -s]; # derivative of transposed rotation matrix

  # here in one go, we go through all landmark measurements vector
  # for each landmark, we assemble
  # the "total" measurement vector, containing all stacked measures
  # the "total" prediction vector, containing all staked predictions
  # the "total" jacobian, consisting of all jacobians of predictions stacked
  # octave allows to "Add" rows to a matrix, thus we dynamically resize them
  
  for i=1:num_landmarks_seen
    % Retrieve info about the observed landmark
    measurement = observations.observation(i);
    z_t(end+1,:) = measurement.x_pose; % where we see the landmark
    z_t(end+1,:) = measurement.y_pose;

    % Absolute true position of landmark
    current_land = searchById(landmarks, measurement.id);
    lx = current_land.x_pose; 
    ly = current_land.y_pose;

    % Where I should see that landmark
    t= [lx-mu_x; ly-mu_y];
    measure_prediction = Rt * t;

    h_t(end+1,:) = measure_prediction(1);
    h_t(end+1,:) = measure_prediction(2);

    % Compute its Jacobian
    C = zeros(2,state_dim);
    C(:,1:2) = -Rt;
    C(:,3) = Rtp * t;

    C_t(end+1,:) = C(1,:);
    C_t(end+1,:) = C(2,:);
  endfor

  % Observation noise
  noise = 0.01;
  sigma_z = noise^2*eye(2*num_landmarks_seen);
  
  % Kalman gain
  K = sigma*C_t'*inv(sigma_z+C_t*sigma*C_t');
  %   [3x3][3x2n]   [2nx2n] [2nx3][3x3][3x2n]

  % Update mu
  mu += K*(z_t-h_t);
  %    [3x2n] [2nx1]

  % Update sigma
  sigma = (eye(state_dim) - K * C_t) * sigma;		
  %              [3x2n][2nx3][3x3]
end
