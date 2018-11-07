close all
clear
clc

addpath '../'
addpath '../tools/g2o_wrapper'
addpath '../tools/visualization'
source "../tools/utilities/geometry_helpers_2d.m"

%load your own dataset dataset
[landmarks, poses, transitions, observations] = loadG2o('datasets/data_bearing_only.g2o');

%% init stuff
%initial pose
mu = rand(3,1)*10-5; 
mu(3) = normalizeAngle(mu(3));
printf('Random initial pose: [%f, %f, %f]\n', mu(1),mu(2), mu(3));
fflush(stdout);

%init covariance
sigma = eye(3)*10;

%init graphics
figure(1); title("ekf-localization-bearing-only");
trajectory = [mu(1), mu(2)];

%simulation cycle
for i=1:length(transitions)
  transitions_t  = transitions(i);
  observations_t = observations(i);

	%predict
	[mu, sigma] = prediction_bearing_only(mu, sigma, transitions_t);

	%correct
	[mu, sigma] = correction_bearing_only(mu, sigma, landmarks, observations_t);

	printf('current pose: [%f, %f, %f]\n', mu(1),mu(2), mu(3));
	fflush(stdout);	

	%plot current situation
	pause(.1);
  	trajectory = [trajectory; mu(1), mu(2)];
	plotState(landmarks, mu, sigma, observations_t, trajectory);
	hold off;

endfor

