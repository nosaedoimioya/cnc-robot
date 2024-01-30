function Q = ur5e_inv_kinematics(x,y,z,phi,theta,psi,frame,q_start)
%{
UR5E_INV_KINEMATICS takes x, y and z axis vectors of the robot end-effector
    position (optionally phi, theta, psi orientation parameters) and 
    returns a [N x numJoints] matrix of the robot joint positions, where N 
    is the number of time steps/positions computed for.

    INPUT(s):
        x - [N x 1] vector of x-axis positions [m]
        y - [N x 1] vector of y-axis positions [m]
        z - [N x 1] vector of z-axis positions [m]
        (OPTIONAL INPUTS)
        phi - [N x 1] vector of roll angle about z-axis [rad]
            default: 0 rad
        theta - [N x 1] vector of pitch angle about y-axis [rad]
            default: 0 rad
        psi - [N x 1] vector of yaw angle about x-axis [rad]
            default: pi/2 rad
        (phi, theta, psi must be defined with respect to the base frame)
        frame - string to specify 'end-effector' or 'base' frame
                for which the x, y, z positions are defined.
                (end-effector positions defined relative to current
                position as a zero postion)
            default: 'base'
        q_start - [numJoints x 1] vector of starting joint positions; used
        to compute starting x,y,z position when end-effector frame is used
        [rad] (must be provided if frame is 'end-effector'
        
    OUTPUT(s):
        Q - the joint positions [q1,q2,q3,q4,q5,q6] as a [N x numJoints] 
            matrix

%}

% Length of vector
N = length(x);

% Check the number of input arguements
if nargin < 7
    frame = 'base';
end

if nargin < 6
    psi = ones(N,1)*(pi/2);
    theta = zeros(N,1);
    phi = zeros(N,1);
end

% Set predefined list of valid frames
predefinedFrames = ["base", "end-effector"];

% Check that the frame was entered as a string
if ~isstring(frame) && ~ischar(frame)
    error(frame + " is not a valid string. Please enter a" + ...
        " string representing the frame of the x-, y-, & z-axis positions." + ...
        " You can enter: %s", strjoin(predefinedFrames, ', '))
end

% Check if the frame is in the predefined list of frames
frameIsValid = ismember(frame, predefinedFrames);
if ~frameIsValid
    error("The frame " + frame + " is not a valid frame. " + ...
        "Enter one of the frames in the list: %s", ...
        strjoin(predefinedFrames, ', '))
end

% Check if all the vectors are the same length
sameLength = ( (N == length(y)) && (N == length(z)) && ...
               (N == length(phi)) && (N == length(theta)) && ...
               (N == length(psi)) );
if ~sameLength
    error("Input vectors are not the same length.")
end

% Constants
numJoints = 6;
numSolutions = 8;

% DH parameters for UR5e
alpha = [pi/2,0,0,pi/2,-pi/2,0]; %[rad]
a = [0,-0.425,-0.3922,0,0,0]; %[m]
d = [0.1625,0,0,0.1333,0.0997,0.0996]; %[m]

% Transformation matrix - returns the transformation matrix at joint n
% relative to joint n-1
A = @ur5e_transformation_matrix;

% We have to change the x, y, and z axis vectors to the base frame if they 
% are defined relative to the end effector frame
if ~strcmp(frame,'base')
    % Check if q_start is defined - note that the end-effector orientations
    % also have to be defined (in world frame) for one to use the end-effector
    % frame to define the positions.
    if nargin < 8
        error("When defining positions in the " + frame + " you must " + ...
            "define the starting joint positions as a Nx1 vector and " + ...
            "include it as an input argument.")
    end

    % Check that q_start is the length of the number of joints for the
    % robot
    if length(q_start) ~= numJoints
        error("The input joint configuration defines " + ...
            length(q_start) + " joint(s), but the robot has " + ...
            numJoints + " joints.")
    end

    % Check if q_start is a column vector
    if size(q_start,1) < size(q_start,2)
        q_start = q_start.';
    end

    col = 1; % column of q_start to use for computation (only one column)
    A1 = A(1,q_start,col,a,d,alpha);
    A2 = A(2,q_start,col,a,d,alpha);
    A3 = A(3,q_start,col,a,d,alpha);
    A4 = A(4,q_start,col,a,d,alpha);
    A5 = A(5,q_start,col,a,d,alpha);
    A6 = A(6,q_start,col,a,d,alpha);
    
    % Transformation matrix from the end-effector frame to the base frame
    T_06 = A1*A2*A3*A4*A5*A6;
end

% Inverse kinematics
Q = zeros(N,numJoints);
for i = 1:N
    % Position and orientation values
    c_phi = cos(phi(i));        s_phi = sin(phi(i));
    c_theta = cos(theta(i));    s_theta = sin(theta(i));
    c_psi = cos(psi(i));        s_psi = sin(psi(i));
    xi = x(i);    yi = y(i);    zi = z(i);

    % Transform position to base frame if frame is not in base
    if ~strcmp(frame,'base')
        new_pos = T_06 * [xi;yi;zi;1];
        xi = new_pos(1); yi = new_pos(2); zi = new_pos(3);
    end

    desired_pos = ...
            [c_phi*c_theta, -s_phi*c_phi+c_phi*s_theta*s_psi, s_phi*s_psi+c_phi*s_theta*c_psi,  xi;
             s_phi*c_theta, c_phi*c_psi+s_phi*s_theta*s_psi,  -c_phi*s_psi+s_phi*s_theta*c_psi, yi;
            -s_theta,       c_theta*s_psi,                    c_theta*c_psi,                    zi;
             0,             0,                                0,                                1];
    
    % Compute possible options for inverse kinematics - [numJoints x 
    % 8 (numSolutions)] matrix
    theta_options = ur5e_algorithm_invKin(desired_pos,numJoints,a,d,alpha);

    % Eliminate unreachable positions
    unreachableSolns = false(1,numSolutions);
    for k = 1:numSolutions
        hasNaN = any(isnan(theta_options(:,k)));
        exceedsLimits = any(abs(theta_options(:,k)) > 2*pi);
        if hasNaN || exceedsLimits
            unreachableSolns(k) = true;
        end
    end
    theta_options = theta_options(:, ~unreachableSolns);
    tempNumSolutions = size(theta_options,2);

    % Check if there are no (0) solutions
    if tempNumSolutions == 0
        error("Unreachable point at position "+ i +". The provided " + ...
            "trajectory has one or more unreachable points. Note that " + ...
            "positions must be entered in meters and orientations in " + ...
            "radians.")
    end

    % Select the joint position from possible solutions
    if i == 1
        % If starting position is provided, select the position closest (by
        % norm) to the starting position
        if ~strcmp(frame,'base')
            diff = theta_options - q_start.';
            norm_diff = zeros(1,tempNumSolutions);
            for j = 1:tempNumSolutions
                norm_diff(j) = norm(diff(:,j));
            end
            [~,minNormIdx] = min(norm_diff);
            Q(i,:) = theta_options(:,minNormIdx);
        else
            % Select solution with the smallest norm (smallest overall
            % motion)
            for j = 1:tempNumSolutions
                norm_diff(j) = norm(theta_options(:,j));
            end
            [~,minNormIdx] = min(norm_diff);
            Q(i,:) = theta_options(:,minNormIdx);
        end

    else
        % Select the joint positions closest (by norm) to the previous 
        % joint positions
        diff = theta_options - Q(i-1,:).';
        norm_diff = zeros(1,tempNumSolutions);
        for j = 1:tempNumSolutions
            norm_diff(j) = norm(diff(:,j));
        end
        [~,minNormIdx] = min(norm_diff);
        Q(i,:) = theta_options(:,minNormIdx);
    end
end


end