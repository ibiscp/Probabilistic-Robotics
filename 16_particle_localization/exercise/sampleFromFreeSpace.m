% this function computes the a set of random sample
% inputs:
%   min_v: minumum value on (x,y,theta)
%   max_v: maximum value on (x,y,theta)
%   dim: number of required samples

% outputs:
%   samples: the randomly chosen samples

function samples = sampleFromFreeSpace(min_v, max_v, dim)

samples = min_v + (max_v - min_v)*rand(3, dim);
samples(3,:) = normalizeAngle(samples(3,:));

endfunction
