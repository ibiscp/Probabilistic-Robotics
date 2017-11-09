% this function computes the resampling of the particle filter
% inputs:
%   samples: the set of particles
%   weights: set of weights
%   dim_samples: how many samples we want to resample

% outputs:
%   new_samples: set of resampled particles
%   new_weights: set of new weights of value (1/dim_samples)

function [new_samples, new_weights] = resample(samples, weights, dim_samples)

	indices = uniformSample(weights', dim_samples);

	new_samples = samples;
	new_weights = ones(1,dim_samples)/dim_samples;

	for i=1:dim_samples
		new_samples(:,i) = samples(:,indices(i));
	endfor

endfunction
