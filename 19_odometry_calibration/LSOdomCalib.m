#import 2d geometry utils
source "../tools/utilities/geometry_helpers_2d.m"

source "exercise/ls_calibrate_odometry.m"
#source "solution/ls_calibrate_odometry.m"

h=figure()

more off;
#load the calibration matrix
disp('loading the matrix');
Z=load("../datasets/ls_calib.dat");

#compute the ground truth trajectory
TrueTrajectory=compute_odometry_trajectory(Z(:,1:3));
disp('ground truth');
hold on;
plot(TrueTrajectory(:,1),TrueTrajectory(:,2), 'r-', 'linewidth', 2);
pause(1);

#compute the uncalibrated odometry
OdomTrajectory=compute_odometry_trajectory(Z(:,4:6));
disp('odometry');
hold on;
plot(OdomTrajectory(:,1),OdomTrajectory(:,2), 'g-', 'linewidth', 2);
pause(1);

disp('computing calibration parameters');
#compute the calibration parameters
X=ls_calibrate_odometry(Z);
disp(X);
pause(1);

disp('computing calibrated odometry');
COdom=apply_odometry_correction(X,Z(:,4:6));
CalTrajectory=compute_odometry_trajectory(COdom);
hold on;
plot(CalTrajectory(:,1),CalTrajectory(:,2), 'b-', 'linewidth', 2);


waitfor(h);