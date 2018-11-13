% this function computes a Uniform Sampling on a given distribution
% inputs:
%   weights: the input distribution
%   num_desired_samples: number of required samples

% outputs:
%   sampled_indices: set of sampled indexes from weights

function sampled_indices = uniformSample(weights, num_desired_samples)

dim_weights = size(weights,1);

%normalize the weights (if not normalized)
normalizer = 1./sum(weights);
%resize the indices
step = 1./num_desired_samples;

y0 = rand()*step; 	%sample between 0 and 1/num_desired_sample;ù
yi = y0;		%value of the sample in the y space
cumulative = 0;		%this ii our running cumulative distribution

for weight_index=1:dim_weights
	cumulative += normalizer*weights(weight_index); %update cumulative
	% fill with current_weight_index
	% until the cumulative does not become larger than yi
	while(cumulative > yi)
		sampled_indices(end+1,1) = weight_index;
		yi += step;
	endwhile
endfor

endfunction
