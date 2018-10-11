#this function implements the kalman prediction step of our localizing robot
# inputs: 
#   transition: is a data structure containing several things,
#               coming from the simulator
#               of these we are interested only in the offset
#               accessible through the "v" field of the structure
#               
#               transition.v(1)=offset x
#               transition.v(2)=offset y (ignore for a diff drive robot)
#               transition.v(3)=offset theta 
#  mu: is the mean of (x,y,theta) the previously estimated robot pose
#  sigma: is the mean of the previously estimated robot pose (3x3 matrix)

# outputs 
# [mu, sigma] are mean and covariance of the estimate after transition

function [mu, sigma] = prediction(mu, sigma, transition)


	u = transition.v;
	%it returns u = [ux, uy, utheta]. simply not consider uy

	%predict mu // this is our f(x,u) function in the slides
	mu = transition_model(mu, u);

	mu_x = mu(1);
	mu_y = mu(2);
	mu_theta = mu(3);

	u_x = u(1);
	u_theta = u(3);

	s = sin(mu_theta);
	c = cos(mu_theta);

	%Jacobian A
	A = [1	0	-u_x*s;
		  0	1	u_x*c;
		  0	0	1] ; 

	%Jacobian B
	B = [c	0;
		  s	0;
		  0	1];

	%motion noise
	noise = 0.1; 			%constant part	
	v_noise = u_x^2;	 	%lin vel dependent part
	w_noise = u_theta^2;		%ang vel dependent part

	sigma_u = [v_noise + noise^2	0;
				0						w_noise + noise^2];

	%predict sigma
	sigma = A*sigma*A' + B*sigma_u*B';

end
