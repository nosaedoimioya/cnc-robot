function A = ur5e_transformation_matrix(n,theta,c,a,d,alpha)
%{
TRANSFORMATION_MATRIX returns the homogeneous transformation matrix for
joint n given the joint position and the Denavit-Hartenberg parameters.
   INPUT(s): 
       n - joint number (row of theta)
       theta - matrix of joint positions [numJoints x numCols]; theta(n,c)
       defines the joint position we are interested in. There can be 1
       column or several columns stored in the matrix
       c - joint column (column of theta)
       a - [1 x numJoints] 'a' vector from the D-H parameters [m]
       d - [1 x numJoints] 'd' vector from the D-H parameters [m]
       alpha - [1 x numJoints] 'alpha'vector from the D-H parameters [rad]
   OUTPUT(s)
       A - [4x4] homogeneous transformation matrix
%}

% Position transformations
T_a = eye(4); % 'a' (x) distance translation
T_a(1,4) = a(1,n);
T_d = eye(4); % 'd' (z) distance translation
T_d(3,4) = d(1,n);

% Rotation transformations
% rotation about the z-axis
Rzt = [cos(theta(n,c)), -sin(theta(n,c)), 0, 0;
       sin(theta(n,c)), cos(theta(n,c)),  0, 0;
       0,            0,             1, 0;
       0,            0,             0, 1];

% rotation about the x-axis
Rxa = [1, 0,               0,               0;
       0, cos(alpha(1,n)), -sin(alpha(1,n)), 0;
       0, sin(alpha(1,n)), cos(alpha(1,n)), 0;
       0, 0,               0,               1];

A = T_d * Rzt * T_a * Rxa;
end