#this function implements the kalman prediction step of our UKF localization system
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
#  mu: is the mean of the robot pose
#  sigma: is the mean of the previously estimated robot pose (3x3 matrix)
#
# outputs 
# [sigmaP, weightsM, weightsC]: sigma points and weights


function [sigmaP, weightsM, weightsC] = prediction(mu, sigma, transition)

	u = transition.v;
        %it returns u = [ux, uy, utheta]. simply not consider uy
	u_x = u(1);
	u_theta = u(3);
	
  %motion noise
  noise = 0.05;                   %constant part  
  v_noise = u_x^2;                %lin vel dependent part
  w_noise = u_theta^2;            %ang vel dependent part

  sigma_u = [ noise+v_noise, 0;
          0, noise+v_noise];

	sigma_xu = zeros(5,5);
	sigma_xu(1:3,1:3) = sigma;
	sigma_xu(4:5,4:5) = sigma_u;

	mu_xu = zeros(5,1);
	mu_xu(1:3) = mu;
	mu_xu(4:5) = [u_x; u_theta];

	% extract the sigma points
	[sigmaP_xu, weightsM, weightsC] = compute_sigma_points(mu_xu, sigma_xu);
	%the returned sigmaP matrix is composed by (2n+1) sigma point, one
	% for each column

	%apply transition to every sigma point
	for i=1:size(sigmaP_xu,1)
		curr_sigma_mu = sigmaP_xu(1:3,i);
		curr_input_u = [sigmaP_xu(4,i); 0; sigmaP_xu(5,i)]; 
		sigmaP(:,end+1) = motion_model(curr_sigma_mu, curr_input_u);
	end

end
