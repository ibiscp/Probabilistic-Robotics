#computes the trajectory of the robot by chaining up
#the incremental movements of the odometry vector
#U:	a Nx3 matrix, each row contains the odoemtry ux, uy utheta
#T:	a Nx3 matrix, each row contains the robot position (starting from 0,0,0)
function T=compute_odometry_trajectory(U)
	T=zeros(size(U,1),3);
	P=v2t(zeros(1,3));
	for i=1:size(U,1),
		u=U(i,1:3)';
		P *= v2t(u);
		T(i,1:3)=t2v(P)';
	end
end

#computes a calibrated vector of odometry measurements
#by applying the bias term to each line of the measurements
#X: 	3x3 matrix obtained by the calibration process
#U: 	Nx3 matrix containing the odometry measurements
#C:	Nx3 matrix containing the corrected odometry measurements	
function C=apply_odometry_correction(X, U)
	C=zeros(size(U,1),3);
	for i=1:size(U,1),
		u=U(i,1:3)';
		uc = X*u;
		C(i,:)=uc;
	end
end

#the measurements are coming in the
#Z matrix

#this function solves the odometry calibration problem
#given a measurement matrix Z.
#Every row of the matrix contains
#z_i = [u'x, u'y, u'theta, ux, uy, ytheta]
#Z:	The measurement matrix
#X:	the calubration matrix
#returns the bias correction matrix BIAS
function X=ls_calibrate_odometry(Z)
	#accumulator variables for the linear system
	H=zeros(9,9);
	b=zeros(9,1);
	#initial solution (the identity transformation)
	X=eye(3); 
	
	#loop through the measurements and update the
	#accumulators
	for i=1:size(Z,1),
		e=error_function(i,X,Z);
		A=jacobian(i,Z);
		H=H+A'*A;
		b=b+A'*e;
	end
	#solve the linear system
	deltaX=-H\b;
	#this reshapes the 9x1 increment vector in a 3x3 atrix
	dX=reshape(deltaX,3,3)';
	#computes the cumulative solution
	X=X+dX;
end

#this function computes the error of the i^th measurement in Z
#given the calibration parameters
#i:	the number of the measurement
#X:	the actual calibration parameters
#Z:	the measurement matrix
#e:	the error of the ith measurement
function e=error_function(i,X,Z)
	uprime=Z(i,1:3)';
	u=Z(i,4:6)';
	e= uprime-X*u;
end

#derivative of the error function for the ith measurement in Z
#does not depend on the state
#i:	the measuement number
#Z:	the measurement matrix
#A:	the jacobian of the ith measurement
function A=jacobian(i,Z)
	u=Z(i,4:6);
	A=zeros(3,9);
	A(1,1:3)= -u;
	A(2,4:6)= -u;
	A(3,7:9)= -u;
end
