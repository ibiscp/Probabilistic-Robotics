% this function computes the update of the weights of the particle filter
% inputs:
%   samples: the set of particles (one for each column)
%   weights: vector of weights
%   landmarks: the set of landmarks in the map
%   observations: current observations 

% outputs:
%   weights: the updated weights

function weights = update(samples, weights, landmarks, observations)

	% determine how many landmarks we have seen
	num_landmarks_seen = length(observations.observation);

	num_particles = size(samples,2);

	num_landmarks = size(landmarks,2);

	%if I've seen no landmarks, i do nothing
	if (num_landmarks_seen==0)
		return;
	endif
	
	sigma_z_noise = 5;
	Sigma_z_noise = [sigma_z_noise^2 0;
			 0 sigma_z_noise^2];
	Omega_z_noise = inv(Sigma_z_noise);

	l_miss = 30;

	for i=1:num_particles
		l_mn = zeros(num_landmarks_seen, num_landmarks);
		particle = samples(:,i);

		for n=1:num_landmarks
			landmark = landmarks(n);
			land_x = landmark.x_pose;
			land_y = landmark.y_pose;
			[curr_h, _] = measurement_function(particle, [land_x; land_y]);

			for m=1:num_landmarks_seen
				measurement = observations.observation(m);
				delta = curr_h - [measurement.x_pose; measurement.y_pose];
				l_mn(m,n) = delta' * Omega_z_noise * delta;
			endfor
		endfor
		
		negative_log_likelihood = min(l_mn');
		dropped_measurements = 0;
		for k=1:num_landmarks_seen
			if(negative_log_likelihood(k) > l_miss)
				negative_log_likelihood(k) = 0;
				dropped_measurements++;
			endif;
		endfor

		sum_of_negative_log_likelihood = sum(negative_log_likelihood);

		weights(i) *= exp(-dropped_measurements*l_miss - sum_of_negative_log_likelihood);

	endfor



endfunction
