function observations = getObservations(map_, row_, col_)
	observations = [0, 0, 0, 0];

  #sampling setup
	minimum_probability    = unifrnd(0, 1);
	cumulative_probability = 0;

	#convert numbers from 0-15 to binary, i.e. (0,0,0,0) to (1,1,1,1)
	for observation = 0:15
		current_observations    = getBinaryArray(observation);
		current_probability     = observationModel(map_, row_, col_, current_observations);
		cumulative_probability += current_probability;
		if(cumulative_probability > minimum_probability)
			observations = current_observations;
			return;
		endif
	endfor
endfunction

