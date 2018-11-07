#this function implements the kalman prediction step of our SLAM system
# inputs: 
#   transition: is a data structure containing several things,
#               coming from the simulator
#               of these we are interested only in the offset
#               accessible through the "v" field of the structure
#               
#               transition.v(1)=offset x
#               transition.v(2)=offset y (ignore for a diff drive robot)
#               transition.v(3)=offset theta 
#
#  considering as 'm' the number of seen landmarks
#  mu: is the mean of (x,y,theta, l1, l2, ..., lm), i.e. the previously estimated robot pose
#      and the m landmark positions
#  sigma: is the mean of the previously estimated robot pose and landmark positions ((3+m)x(3+m) matrix)

# outputs 
# [mu, sigma] are mean and covariance of the estimate after transition

function [mu, sigma] = prediction(mu, sigma, control_input)

  # Domain spaces
  dimension_mu = size(mu, 1);
  dimension_u  = 2;

  # Get the control input u = [ux, uy, utheta]
  u = control_input.v;

  # Predict the robot motion
  mu_r = transition_model(mu(1:3), u);

  # Update the robot state
  mu(1:3) = mu_r;

  # Readability: current pose
  mu_x     = mu_r(1);
  mu_y     = mu_r(2);
  mu_theta = mu_r(3);

  # Readability: current control input
  u_x     = u(1); #translational velocity
  u_theta = u(3); #rotational velocity

  # Jacobian A: df(x, u)/dx
  A = eye(dimension_mu);
  A(1,3) = -u_x*sin(mu_theta);
  A(2,3) = u_x*cos(mu_theta);

  #Jacobian B: df(x, u)/du
  B = zeros(dimension_mu, dimension_u);
  B(1:3,:) = [cos(mu_theta) 0;
              sin(mu_theta) 0;
                  0         1];

  # Control noise u: standard deviations
  sigma_u = 0.1;        # Constant part
  sigma_T = u_x^2;      # Translational velocity dependent part
  sigma_R = u_theta^2;  # Rotational velocity dependent part

  # Compose control noise covariance sigma_u
  sigma_u = diag(sigma_T+sigma_u^2, sigma_R+sigma_u^2);

  # Predict sigma
  sigma = A * sigma * A' + B * sigma_u * B';
  #     [nxn] [nxn] [nxn] [nx2][2x2][2xn]
endfunction

