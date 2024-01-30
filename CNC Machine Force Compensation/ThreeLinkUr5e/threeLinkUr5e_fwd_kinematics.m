function [X,O] = threeLinkUr5e_fwd_kinematics(Q)
%{
UR5E_FWD_KINEMATICS takes a [N x numJoints] matrix of the robot joint
    positions and returns a [N x 3] matrix of the end-effector positions 
    and [N x 3] matrix of end-effector orientations, where N is the number
    of time steps/positions computed for.
    
    INPUT(s):
        Q - the joint positions [q1,q2,q3] as a [N x numJoints] 
            matrix
    OUTPUT(s):
        X - [N x 3] matrix of x-, y-, and z-axis end-effector positions
            [x, y, z]
            x - [N x 1] vector of x-axis positions [m]
            y - [N x 1] vector of y-axis positions [m]
            z - [N x 1] vector of z-axis positions [m]
        O - [N x 3] matrix of end-effector orientations
            [phi, theta, psi]
            phi - [N x 1] vector of roll angle about z-axis [rad]
                default: 0 rad
            theta - [N x 1] vector of pitch angle about y-axis [rad]
                default: 0 rad
            psi - [N x 1] vector of yaw angle about x-axis [rad]
                default: pi/2 rad

%}

numJoints = 3;
N = size(Q,1);

% Check that we have the right number of joints
if size(Q,2) ~= numJoints
    error("The input matrix has " + size(Q,2) + " joint(s) but the " + ...
        "function expected " + numJoints + " joints.")
end

% Constants

% DH parameters for UR5e
alpha = [pi/2,0,0,0]; %[rad]
a = [0,-0.425,-0.3922,0]; %[m]
d = [0.1625,0,0,0.1333]; %[m]

% Transformation matrix - returns the transformation matrix at joint n
% relative to joint n-1
A = @threeLinkUr5e_transformation_matrix;

% Compute forward kinematics
X = zeros(N,3);
O = zeros(N,3);

% Add a fouth column (joint) for the translation of the mass
Q = [Q, zeros(N,1)];
% Traspose Q for to comply with the form that A expects
Q = Q.'; % now [numJoints x N] matrix

% default orientation
default_O = [0;0;pi/2]; % [rad]

for col = 1:N
    A1 = A(1,Q,col,a,d,alpha);
    A2 = A(2,Q,col,a,d,alpha);
    A3 = A(3,Q,col,a,d,alpha);
    A4 = A(4,Q,col,a,d,alpha);
    
    % Transformation matrix from the end-effector frame to the base frame
    T_04 = A1*A2*A3*A4;

    X(col,1) = T_04(1,4);
    X(col,2) = T_04(2,4);
    X(col,3) = T_04(3,4);
    
    % Rotation matrix (XYZ-Euler Angle Transformation)
    R_04 = T_04(1:3,1:3);

    % Return the two orientation options
    orientation_options = threeLinkUr5e_rotationMatrixToEulerAngles(R_04);

    % Select the smallest change (by norm) in orientation angles
    if col == 1
        diff = orientation_options - default_O;
        norm_diff = zeros(1,size(orientation_options,2));
        for j = 1:size(orientation_options,2)
            norm_diff(j) = norm(diff(:,j));
        end
        [~,minNormIdx] = min(norm_diff);
        O(col,:) = orientation_options(:,minNormIdx);
    else
        % Select the Euler angles closest (by norm) to the previous 
        % Euler anglers
        diff = orientation_options - O(col-1,:).';
        norm_diff = zeros(1,size(orientation_options,2));
        for j = 1:size(orientation_options,2)
            norm_diff(j) = norm(diff(:,j));
        end
        [~,minNormIdx] = min(norm_diff);
        O(col,:) = orientation_options(:,minNormIdx);
    end
end

end