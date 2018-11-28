# Calculate displacement given odometry
function state = displacement(parameters, measurement)
  state = zeros(1,3);
  
  l = parameters(1) * measurement(1);
  r = parameters(2) * measurement(2);
  b = parameters(3);
  
  delta_P = (r + l);
  delta_M = (r - l);
  
  delta_T = delta_M/b;
  
  # Taylor expansion
  P_x = 1 - delta_T^2/6 + delta_T^4/120 - delta_T^6/5040;
  P_y = delta_T/2 - delta_T^3/24 + delta_T^5/720;

  delta_X = delta_P/2 * P_x;
  delta_Y = delta_P/2 * P_y;

  state(1) = delta_X;
  state(2) = delta_Y;
  state(3) = delta_T;
  
end

# Computes the trajectory by chaining up incremental movements of the ground truth
function T=compute_incremental_trajectory(U)
	T=zeros(size(U,1),3);
	P=v2t(zeros(1,3));
	for i=1:size(U,1),
		u=U(i,:)';
		P *= v2t(u);
		T(i,1:3)=t2v(P)';
	end
end

# Computes trajectory by chaining up incremental movements of the odometry
function T=compute_odometry_trajectory(parameters, U)
	T=zeros(size(U,1),3);
	P=v2t(zeros(1,3));
	for i=1:size(U,1),
		u=U(i,1:2)';
    state = displacement(parameters, u);
		P *= v2t(state);
		T(i,1:3)=t2v(P)';
	end
end

# Calculate calibrated values of the parameters
function X=ls_calibrate_odometry(parameters,Z)
	# Accumulator variables for the linear system
  H=zeros(3,3);
	b=zeros(3,1);
	X=parameters; # Initial solution
	
	# Loop through the measurements and update the accumulators
	for i=1:size(Z,1),
		e=error_function(parameters,Z(i,:));
		A=jacobian(parameters,Z(i,1:2));
		H += A'*inv(eye(3))*A;
		b += A'*inv(eye(3))*e';
	end
	deltaX=-H\b;  # Solve the linear system
	X=X+deltaX';  # Computes the cumulative solution
end

# Computes the error of the ith measurement in Z given the calibration parameters
function e=error_function(parameters,Z)
	uprime=Z(1:2);
	state=Z(3:5);
  u = displacement(parameters, uprime);
	e = state - u;
end

# Derivative of the error function for the ith measurement in Z (does not depend on the state)
function A=jacobian(parameters, measurement)
  epsilon = 0.05;
  kl = parameters(1);
  kr = parameters(2);
  b = parameters(3);

  % Numerical jacobian:
  displacement_kl = displacement([kl + epsilon, kr, b], measurement) - displacement([kl - epsilon, kr, b], measurement);
  displacement_kr = displacement([kl, kr + epsilon, b], measurement) - displacement([kl, kr - epsilon, b], measurement);
  displacement_b  = displacement([kl, kr, b + epsilon], measurement) - displacement([kl, kr, b - epsilon], measurement);
  A = -[displacement_kl; displacement_kr; displacement_b]' / (2*epsilon);
end
