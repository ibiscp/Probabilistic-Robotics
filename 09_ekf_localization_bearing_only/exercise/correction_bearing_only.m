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
#            - the bearing angle where we have seen the landmark (theta) w.r.t the robot
#outputs:
#  [mu, sigma]: the updated mean and covariance

function [mu, sigma] = correction_bearing_only(mu, sigma, landmarks, observations)

  % Determine how many landmarks we have seen
  num_landmarks_seen = length(observations.observation);
  
  % Dimension of the state in dim, in our case is fixed to 3
  state_dim = size(mu,1);	

  % If I've seen no landmarks, i do nothing
  if (num_landmarks_seen==0)
    return;
  endif

  # we precompute some quantities that come in handy later on
  mu_x = mu(1);
  mu_y = mu(2);
  mu_theta = mu(3);
  c=cos(mu_theta);
  s=sin(mu_theta);
  R = [c -s; s c];    # rotation matrix
  Rt = [c s;-s c];    # transposed rotation matrix
  Rtp = [-s c;-c -s]; # derivative of transposed rotation matrix
		
  # here in one go, we go through all landmark measurements vector
  # for each landmark, we assemble
  # the "total" measurement vector, containing all stacked measures
  # the "total" prediction vector, containing all staked predictions
  # the "total" jacobian, consisting of all jacobians of predictions stacked
  # octave allows to "Add" rows to a matrix, thus we dynamically resize them

  for i=1:num_landmarks_seen

    % Retrieve info about the observed landmark
    measurement = observations.observation(i);
    z_t(end+1,:) = measurement.bearing; % Where we see the landmark

    % Absolute true position of landmark
    current_land = searchById(landmarks, measurement.id);
    lx = current_land.x_pose;
    ly = current_land.y_pose;

    % Where I should see that landmark
    t= [lx-mu_x; ly-mu_y];
    h = Rt * t;
    bearing_prediction = atan2(h(2), h(1));

    h_t(end+1,:) = bearing_prediction;

    % Compute its Jacobian
    C = 1/(h(1)^2+h(2)^2)*[-h(2) h(1)]*[-Rt Rtp*t];

    C_t(end+1,:) = C;

  endfor

  % Observation noise
  noise = 0.01;
  sigma_z = eye(num_landmarks_seen)*noise^2;

  % Kalman gain
  K = sigma * C_t' * inv(sigma_z + C_t * sigma * C_t');
  %   [3x3]   [3xn]       [nxn]   [nx3]  [3x3]  [3xn]

  % Update mu
  mu += K * (z_t - h_t);
  %    [3xn][nx1] [nx1]

  % Update sigma
  sigma = (eye(state_dim) - K * C_t) * sigma;
  %           [3x3]       [3xn][nx3]   [3x3]

end
