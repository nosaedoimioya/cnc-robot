function theta = ur5e_algorithm_invKin(T_06,numJoints,a,d,alpha)
%{
UR5E_ALGORITHM_INVKIN takes in the homogenous matrix that represents the
position and orientation of the end-effector with respect to the base and
returns a [numJoints x 8] matrix with the possible inverse
kinematics solutions.
   INPUT(s):
       T_06 - [4x4] matrix containing the position and orientation of the
              end-effector with respect to the base 
       numJoints - number of robot joints
       a - [1 x numJoints] 'a' vector from the D-H parameters [m]
       d - [1 x numJoints] 'd' vector from the D-H parameters [m]
       alpha - [1 x numJoints] 'alpha'vector from the D-H parameters [rad]
   OUTPUTS(s):
       theta - [numJoints x 8] matrix with the possible inverse
       kinematics solutions [rad]
%}

A = @ur5e_transformation_matrix;
numSolutions = 8;

theta = zeros(numJoints,numSolutions);

% Position vector of the 5th coordinate frame
P_05 = (T_06 * [0;0;-d(6);1]) - [0;0;0;1];

% theta1
psi = atan2(P_05(2,1), P_05(1,1));
phi = acos(d(4) / sqrt(P_05(2,1)^2 + P_05(1,1)^2));

% The two solutions for theta1 correspond to the shoulder either being left
% or right
theta(1,1:numSolutions/2) = pi/2 + psi + phi;
theta(1,(numSolutions/2)+1:numSolutions) = pi/2 + psi - phi;
theta = real(theta);

% theta5
cl = [1, 5]; % for wrist up or down
for i=1:length(cl)
    c = cl(i);
    T_10 = inv(A(1,theta,c,a,d,alpha));
    T_16 = T_10 * T_06;
    theta(5,c:c+1) = real( acos( (T_16(3,4)-d(4)) / d(6) ) );
    theta(5,c+2:c+3) = -real( acos( (T_16(3,4)-d(4)) / d(6) ) );
end

% theta6
% *** theta6 is not well defined when sin(theta5) = 0 or when T_61(2,4),
% T_61(3,4) = 0
cl = [1, 3, 5, 7]; % for wrist up or down
for i=1:length(cl)
    c = cl(i);
    T_10 = inv(A(1,theta,c,a,d,alpha));
    T_61 = inv(T_10 * T_06);
    if sin(theta(5,c)) == 0 || T_61(1,3) == 0 || T_61(2,3) == 0
        theta(6,c:c+1) = 0;
    else
        theta(6,c:c+1) = real( atan2(-T_61(2,3) / sin(theta(5,c)),...
                               T_61(1,3) / sin(theta(5,c)) ) );
    end
end

% theta3 - elbow up or down
cl = [1, 3, 5, 7]; 
for i=1:length(cl)
    c = cl(i);
    T_10 = inv(A(1,theta,c,a,d,alpha));
    T_56 = A(6,theta,c,a,d,alpha);
    T_45 = A(5,theta,c,a,d,alpha);
    T_14 = (T_10 * T_06) * inv(T_45 * T_56);
    P_13 = (T_14 * [0;-d(4);0;1]) - [0;0;0;1];
    % Check that the position does not violate the triangle inequality
    if abs(a(2))+abs(a(3)) > norm(P_13)
        t3 = acos( ( norm(P_13)^2 - a(2)^2 - a(3)^2 ) / (2 * a(2) * a(3)) );
        theta(3,c) = real(t3);
        theta(3,c+1) = -real(t3);
    else
        error("Unreachable point. The provided " + ...
            "trajectory has one or more unreachable points. Note that " + ...
            "positions must be entered in meters and orientations in " + ...
            "radians.")
    end
end

% theta2 and theta 4
cl = [1, 2, 3, 4, 5, 6, 7, 8]; 
for i=1:length(cl)
    c = cl(i);
    T_10 = inv(A(1,theta,c,a,d,alpha));
    T_65 = inv(A(6,theta,c,a,d,alpha));
    T_54 = inv(A(5,theta,c,a,d,alpha));
    T_14 = (T_10 * T_06) * (T_65 * T_54);
    P_13 = (T_14 * [0;-d(4);0;1]) - [0;0;0;1];

    % theta2
    theta(2,c) = real( -atan2(P_13(2,1), -P_13(1,1)) +...
                        asin(a(3) * sin(theta(3,c)) / norm(P_13) ) );
    % theta4
    T_32 = inv(A(3,theta,c,a,d,alpha));
    T_21 = inv(A(2,theta,c,a,d,alpha));
    T_34 = T_32 * T_21 * T_14;
    theta(4,c) = real( atan2( T_34(2,1), T_34(1,1) ) );
end

end