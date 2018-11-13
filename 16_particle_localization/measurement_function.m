% compute the measurement function for a point based observation
% 
% in case of a differential drive robot, ignore uy (the case seen in the classroom)
% inputs:
%   state_dim: total state dimension (robot + landmark)
%   mu_robot: current robot pose estimate (x,y,theta)
%   absolute_landmark_position: current landmark position in world coordinate 
%   landmark_state_vector_index: index of the landmark in the mu vector
%
% outputs:
%   measurement_prediction: prediction of the landmark in mu_robot frame
%   C_m: Jacobian wrt robot and wrt landmark for kalman filter correction

function [measurement_prediction, C_m] = measurement_function(mu_robot, absolute_landmark_position)

	robot_t = mu_robot(1:2);
	theta = mu_robot(3);

	c=cos(theta);
	s=sin(theta);

	Rt=[c,s;-s c];    #transposed rotation matrix
	Rtp=[-s,c;-c,-s]; #derivative of transposed rotation matrix	

	%where I predict i will see that landmark
	delta_t = absolute_landmark_position - robot_t;
	measurement_prediction = Rt* delta_t;
	
	%init Jacobian
	C_m = zeros(2, 3);

	%Jacobian w.r.t robot
	C_m(1:2,1:2)=-Rt;
	C_m(1:2,3)=Rtp*delta_t;

end
