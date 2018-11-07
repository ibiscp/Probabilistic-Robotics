close all
clear
clc

#load dependencies
addpath "../../"
addpath "../tools/g2o_wrapper"
addpath "../tools/visualization"
source "../tools/utilities/geometry_helpers_2d.m"

#addpath "./exercise" #uncomment this line to target the exercise
addpath "./solution" #comment this line to target the exercise

%load dataset
[landmarks, poses, transitions, observations] = loadG2o('../datasets/dataset_point.g2o');

%% init stuff
%initial pose
mu =  rand(3,1)*10-5; 
mu(3) = normalizeAngle(mu(3));
printf('Random initial pose: [%f, %f, %f]\n', mu(1),mu(2), mu(3));
fflush(stdout);

%init covariance
sigma = eye(3)*0.001;

#init graphics
figure(1); title("ukf-localization");
trajectory = [mu(1), mu(2)];

#simulation cycle
for i=1:length(transitions)

  #obtain transition
  transition_t = transitions(i);

	#predict
	[sigmaP, wM, wC] = prediction(mu, sigma, transition_t);

  #obtain observations
  observations_t = observations(i);

	#correct
	[mu, sigma] = correction(sigmaP, wM, wC, landmarks, observations_t);

	printf('current pose: [%f, %f, %f] observations: %u\n', mu(1),mu(2), mu(3), length(observations_t.observation));
  trajectory = [trajectory; mu(1), mu(2)];
	fflush(stdout);

	#plot current situation
	pause(.1)
	plotState(landmarks, mu, sigma, observations_t, trajectory);
	hold off;
endfor

