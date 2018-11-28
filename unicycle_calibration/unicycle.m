# Import 2d geometry utils
source "../tools/utilities/geometry_helpers_2d.m"
source "calibrate.m"

# Inicial parameters
parameters = [-1, 1, 0.3];

h=figure();

# Load the calibration matrix
disp('loading the matrix');
Z=load("../datasets/differential_drive_calibration_data/differential_drive_calibration.txt");

# Compute the ground truth trajectory
TrueTrajectory=compute_incremental_trajectory(Z(:,3:5));
disp('ground truth');
hold on;
plot(TrueTrajectory(:,1),TrueTrajectory(:,2), 'r-', 'linewidth', 2);
pause(1);

# Compute the uncalibrated odometry
OdomTrajectory=compute_odometry_trajectory(parameters, Z(:,1:2));
disp('odometry');
hold on;
plot(OdomTrajectory(:,1),OdomTrajectory(:,2), 'g-', 'linewidth', 2);
pause(1);

# Compute the calibration parameters
disp('computing calibration parameters');
new_parameters=ls_calibrate_odometry(parameters,Z);
disp(new_parameters);
pause(1);

# Compute the calibrated odometry
disp('computing calibrated odometry');
CalTrajectory=compute_odometry_trajectory(new_parameters,Z(:,1:2));
hold on;
plot(CalTrajectory(:,1),CalTrajectory(:,2), 'b-', 'linewidth', 2);

waitfor(h);