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

function [mu, sigma] = correction(sigmaP, weightsM, weightsC, landmarks, observations)

	% determine how many landmarks we have seen
	num_landmarks_seen = length(observations.observation);

	% dimension of the state in dim, in our case is fixed to 3
	state_dim = size(sigmaP,1);

	%if I've seen no landmarks, i do nothing
	if (num_landmarks_seen==0)
		return;
	endif

	%retrieve the index of observed landmarks
	id_x =1;
	for i=1:num_landmarks_seen
		%retrieve info about the observed landmark	
		measurement = observations.observation(i);

		z_t(end+1,:) = measurement.x_pose; % where we see the landmark
		z_t(end+1,:) = measurement.y_pose;

		current_land = searchById(landmarks, measurement.id);
		lx = current_land.x_pose;
		ly = current_land.y_pose;

		%where I should see that landmark from each sigma point
		for j=1:size(sigmaP,2)
			current_sigma_point = sigmaP(:,j);
			measure_prediction = measurement_function(current_sigma_point, [lx; ly]);
			predicted_sigma_z(id_x:id_x+1,j) = measure_prediction;
		endfor
		id_x +=2;

	endfor

	%once we have computed our predicted_sigma_z, we can reconstruct h_t and relative covariace
	[mu_z, Sigma_z] = reconstruct_mean_cov(predicted_sigma_z, weightsM, weightsC);
	% here we need to correct mu and sigma, so first of all we reconstruct mu and sigma from sigma points
	[mu, sigma] = reconstruct_mean_cov(sigmaP, weightsM, weightsC);

	%observation noise
	noise = 0.01;
	Sigma_noise = eye(2*num_landmarks_seen)*noise;

	Sigma_xz = zeros(state_dim, 2*num_landmarks_seen);
	for i=1:size(sigmaP,2)
		delta_x   = sigmaP(:,i)-mu; # [state_dim x 1]
		delta_z   = predicted_sigma_z(:,i)-mu_z; # [2*num_landmarks_seen x 1]
		Sigma_xz += weightsC(i)*delta_x*delta_z'; # [1 x 1] * [state_dim x 1] * [1 x 2*num_landmarks_seen]
	end

	%Kalman gain
	K = Sigma_xz * inv(Sigma_z + Sigma_noise);

	%update mu
	error      = z_t - mu_z;
	correction = K * error;
	mu = mu + correction;

	%update sigma
	sigma = sigma - K*(Sigma_z + Sigma_noise)*K';
end
