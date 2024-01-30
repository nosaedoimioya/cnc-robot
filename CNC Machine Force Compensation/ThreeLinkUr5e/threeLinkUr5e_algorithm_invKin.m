function theta = threeLinkUr5e_algorithm_invKin(T_04,numJoints,a,d,alpha)
%{
THREELINKUR5E_ALGORITHM_INVKIN takes in the homogenous matrix that represents the
position and orientation of the end-effector with respect to the base and
returns a [numJoints x 8] matrix with the possible inverse
kinematics solutions.
   INPUT(s):
       T_04 - [4x4] matrix containing the position and orientation of the
              end-effector with respect to the base 
       numJoints - number of robot joints
       a - [1 x numJoints] 'a' vector from the D-H parameters [m]
       d - [1 x numJoints] 'd' vector from the D-H parameters [m]
       alpha - [1 x numJoints] 'alpha' vector from the D-H parameters [m]
   OUTPUTS(s):
       theta - [numJoints x 8] matrix with the possible inverse
       kinematics solutions [rad]
%}

numSolutions = 4;

theta = zeros(numJoints,numSolutions);

% Position vector of the 4rd coordinate frame
P_04 = (T_04 * [0;0;-d(4);1]) - [0;0;0;1];

% theta1
psi = atan2(P_04(2,1), P_04(1,1));
% Check for 360 deg rotation
if abs(psi - 2*pi) < 0.0001
    psi = 0;
end


% The two solutions for theta1 correspond to the shoulder either being left
% or right
theta(1,1:numSolutions/2) = psi;
theta(1,(numSolutions/2)+1:numSolutions) = psi - pi;
theta = real(theta);

% theta3 - elbow up or down
cl = [1, 3];
norm_P_13 = (P_04(2,1)^2 + P_04(1,1)^2 - d(2)^2) + (P_04(3,1) - d(1))^2;
D = (norm_P_13 - a(2)^2 - a(3)^2) / (2 * a(2) * a(3));
for i=1:length(cl)
    c = cl(i);
    % Check that the position does not violate the triangle inequality
    if abs(a(2))+abs(a(3)) > norm_P_13
        theta(3,c) = real( atan2(D, sqrt(1-D^2) ) );
        theta(3,c+1) = real( atan2(D, -sqrt(1-D^2) ) );
    else
        error("Unreachable point. The provided " + ...
            "trajectory has one or more unreachable points. Note that " + ...
            "positions must be entered in meters and orientations in " + ...
            "radians.")
    end
end


% theta2
cl = [1, 2, 3, 4]; 
for i=1:length(cl)
    c = cl(i);
    % theta2
    theta(2,c) = real( atan2( sqrt(P_04(2,1)^2 + P_04(1,1)^2 - d(2)^2), ...
                             ( P_04(3,1) - d(1) ) ) - ...
                        atan2(a(2) + a(3)*cos(theta(3,c)), a(3) * sin (theta(3,c)) ) );
end

end