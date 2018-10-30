# retrieve the data associations
# 
# inputs:
#   mu: current mean
#   sigma: current covariance
#   observations: current observation set, from G2oWrapper
#   state_to_id_map: mapping vector state_position-id
#   heuristics: may be used to input the desired level of heuristics
#
# outputs:
#   observations: with the respective, aka associated, ids

function observations = associateLandmarkIDs(mu, sigma, observations, state_to_id_map)
	measurements = observations.observation;

	#determine how many landmarks we have seen in this step
	M = length(measurements);

	#set all the observations to unknown
	for i=1:M
		observations.observation(i).id = -1;
	endfor

	# dimension of the state (robot pose + landmark positions)
	state_dim = size(mu, 1);

	# dimension of the current map (how many landmarks we have already seen)
	N = (state_dim-3)/2;

	#if we do not see any landmarks in the state to associate or have no measurements 
	if(N == 0 || M == 0)
		return;
	endif

	#readability
	mu_landmarks = mu(4:end);
	mu_robot     = mu(1:3);

	#build the association matrix [(current observations) x (number of landmarks)] - with maximum distance values
	A = ones(M, N)*1e3;
	
  #state iterator (we will use this to iterate over the landmarks in the state)
  id_state = 1;

	#now we have to populate the association matrix
	for n=1:N

		#extract landmark from mu
		mu_curr_landmark = mu_landmarks(id_state:id_state+1);
	
		#retrieve measurement predicition
		[measurement_prediction, C] = measurement_function(state_dim, mu_robot, mu_curr_landmark, id_state);
    
    #compute the covariance and its inverse
		sigma_zx = eye(2,2) * 0.01; # minimum uncertainty (check dimensions)
		sigma_nn = C * sigma * C' + sigma_zx; # covariance for the current landmark
    # [2x2] [2x2n+3][2n+3x2n+3][2n+3x2] 

    #compute information matrix
		omega_nn = inv(sigma_nn); # canonical representation of sigma_nn

    #for all landmarks
		for m=1:M

			#obtain current measurement
      measurement = measurements(m);
			z = [measurement.x_pose; measurement.y_pose];

			#compute likelihood for this measurement and landmark
      delta = z-measurement_prediction;
			A(m, n) = delta' * omega_nn * delta; # likelihood with information matrix (omega)
      # [1x1] = [1x2]  *   [2x2]  * [2x1]
      endfor

    #move to the next landmark in the state
		id_state += 2;
	endfor

	#now associate the measurement to the most promising landmark
	# proposed associations will be stored in a [Mx3] matrix composed
	# in this way
	#
	#	[measurement id, proposed landmark id , association matrix value a_mn] 	
	#
	# we will populate such a matrix with the associations surviving
	# the gating heuristic for each step
	# 
	# In the best friends and lonely best friend heuristics we will keep
	# the association as is in case of success, otherwise we will put
	# an id=0 to that measurment, meaning that the association is doubtful

	#configuration heuristics
	gating_tau                         = 1;
  lonely_best_friend_gamma_threshold = 1e-3;

	#1. Gating
  for m=1:M

		#return the min and index on the 'm-th' row
		[a_mn, min_index] = min(A(m,:));

    #if the association passes the gate
		if(a_mn < gating_tau)# add gating condition for this association
		
			#add the possible association - creating the associations vector
      #[measurement id, proposed landmark id , association matrix value a_mn] 
			associations(end+1,:) = [m min_index a_mn]; # add the information to our bookkeeping
		endif
  endfor

	#associations that survived the gating
	number_of_gated_associations = size(associations, 1);

	#2. Best friends
	for i=1:number_of_gated_associations
		a_mn                 = associations(i, 3);
		proposed_landmark_id = associations(i, 2);

    #compute column minimum
		min_on_column = min(A(:, proposed_landmark_id));

    #if the association is not the minimum in the column
		if(min_on_column != a_mn)# check if it is NOT a best friend
			associations(i, 2) = 0; #discard association, it is doubtful
		endif
	endfor
	
	#3. Lonely best friend
  number_of_valid_associations = 0;
	if(M > 1)
		for i=1:number_of_gated_associations
			a_mn                 = associations(i, 3);
			measurement_id       = associations(i, 1);
			proposed_landmark_id = associations(i, 2);

      #this association is doubtful, skip evaluation
			if(proposed_landmark_id == 0)
				continue;
			endif

			# Obtain second best(aka min) value of the row
			ordered_row          = unique(A(measurement_id,:));
			second_row_min_value = ordered_row(2);

			# Obtain second best(aka min) value of the column
			ordered_col          = unique(A(:,proposed_landmark_id));
			second_col_min_value = ordered_col(2);

			# Check if the association is ambiguous
			if((second_row_min_value - a_mn) < lonely_best_friend_gamma_threshold ||
          (second_col_min_value - a_mn) < lonely_best_friend_gamma_threshold)
				associations(i,2) = 0; # Discard association, it is doubtful
			else
        
        # Valid association increment
        ++number_of_valid_associations;
      endif
		endfor
	endif

	#assign the associations to the observations
	for i=1:number_of_gated_associations
		measurement_id       = associations(i, 1);
		proposed_landmark_id = associations(i, 2);		

		##assign the proposed association OR 0 (a magic number that indicates no landmark was found)
		observations.observation(measurement_id).id = proposed_landmark_id;
	endfor

  #info
  printf("valid associations: %u / measurements: %u / landmarks: %u\n", number_of_valid_associations, M, N);
end
