function euler_angles = threeLinkUr5e_rotationMatrixToEulerAngles(rotation_matrix)
%{
THREELINKUR5E_ROTATIONMATRIXTOEULERANGLES takes a [3 x 3] rotation matrix and
    returns a [3 x 2] matrix of the euler angles representing that matrix
    based on the XYZ-Euler Angle Transformation 
    Note that there are 2 solutions (that might be the same)
    
    INPUT(s):
        rotation_matrix - the [3x3] rotation matrix 
    OUTPUT(s):
        X - [N x 3] matrix of x-, y-, and z-axis end-effector positions
            [x, y, z]
            x - [N x 1] vector of x-axis positions [m]
            y - [N x 1] vector of y-axis positions [m]
            z - [N x 1] vector of z-axis positions [m]
        euler_angles - [3 x 2] matrix of Euler angles
            [phi (roll); 
            theta (pitch); 
            psi (yaw)]
            phi - [1 x 2] vector of roll angle about z-axis [rad]
            theta - [1 x 2] vector of pitch angle about y-axis [rad]
            psi - [1 x 2] vector of yaw angle about x-axis [rad]
                default: pi/2 rad

%}
    % Check if the input is a 3x3 rotation matrix
    if size(rotation_matrix) ~= [3, 3]
        error('Input is not a valid 3x3 rotation matrix.');
    end
    
    % Extract individual rotation angles (Euler angles)
    % Calculate pitch (rotation around Y-axis)
    pitch1 = -asin(rotation_matrix(3, 1));
    pitch2 = pi + asin(rotation_matrix(3, 1));
    
    % Calculate yaw (rotation around Z-axis) and roll (rotation around 
    % X-axis) for both solutions
    if abs(rotation_matrix(3, 1)) ~= 1
        % Calculate yaw (rotation around Z-axis)
        yaw1 = atan2(rotation_matrix(3, 2) / cos(pitch1), rotation_matrix(3, 3) / cos(pitch1));
        yaw2 = atan2(rotation_matrix(3, 2) / cos(pitch2), rotation_matrix(3, 3) / cos(pitch2));
        % Calculate roll (rotation around X-axis)
        roll1 = atan2(rotation_matrix(2, 1) / cos(pitch1), rotation_matrix(1, 1) / cos(pitch1));
        roll2 = atan2(rotation_matrix(2, 1) / cos(pitch2), rotation_matrix(1, 1) / cos(pitch2));
    else
        % Handle the singularity case when pitch is near ±90 degrees
        % Yaw and roll are undefined
        % Set yaw to the default pi/2 value, calculate roll
        if rotation_matrix(3, 1) == -1 % pitch = pi/2
            yaw1 = pi/2;
            yaw2 = pi/2;
            roll1 = yaw1 - atan2(rotation_matrix(1,2),rotation_matrix(1,3));
            roll2 = yaw2 - atan2(rotation_matrix(1,2),rotation_matrix(1,3));
        else % pitch = -pi/2
            yaw1 = pi/2;
            yaw2 = pi/2;
            roll1 = atan2(-rotation_matrix(1,2),-rotation_matrix(1,3)) - yaw1;
            roll2 = atan2(-rotation_matrix(1,2),-rotation_matrix(1,3)) - yaw2;
        end
    end

    % Collect the Euler angles
    euler_angles = [roll1,  roll2;
                    pitch1, pitch2;
                    yaw1,   yaw2];
end