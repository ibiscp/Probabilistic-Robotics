close all
clear
clc

addpath '../'
addpath '../tools/g2o_wrapper'
addpath '../tools/visualization'
source "../tools/utilities/geometry_helpers_2d.m"

addpath "./exercise" % uncomment this line to target the exercise
%addpath "./solution"

% load your own dataset dataset
[landmarks, poses, transitions, observations] = loadG2o('datasets/dataset_point.g2o');

% how many sample we want to deal with
dim_samples = 2000;

% map dimensions specified to sample the initial particles
map_min = -12;
map_max = 12;

% uniformly sample from free space
samples = sampleFromFreeSpace(map_min, map_max, dim_samples);
% init weights to 1
weights = ones(1,dim_samples);

% keep track of the best particle (with highest weight) to visualize it
best_particle_id = 0;

% simulation cycle
for i=1:length(transitions)
	gt_pose = poses(i);

%t = cputime;
	% predict
        samples = prediction(samples, transitions(i));
%printf("prediction time: %f\n", cputime - t);
        
%t = cputime;
	% plot the state
        plotStatePF(samples, weights, landmarks, best_particle_id, gt_pose);
%printf("plotting time: %f\n", cputime - t);
        
%t = cputime;
	% update the weights
        weights = update(samples, weights, landmarks, observations(i));
%printf("update time: %f\n", cputime - t);

	% check for the best particle
        [_, best_particle_id] = max(weights);
%t = cputime;
	% resampling
       [samples, weights] = resample(samples, weights, dim_samples);
%printf("resampling time: %f\n", cputime - t);

        fflush(stdout);
        plotStatePF(samples, weights, landmarks, best_particle_id, gt_pose);
        
endfor

