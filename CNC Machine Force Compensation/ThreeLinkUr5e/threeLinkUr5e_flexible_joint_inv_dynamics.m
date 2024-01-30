function [Q_dot] = threeLinkUr5e_flexible_joint_inv_dynamics(t,Q,td,Qd,robot)
%{
THREELINKUR5E_FLEXIBLE_JOINT_INV_DYNAMICS This function simulates the closed loop 
flexible body dynamics of a 6 DOF UR5e robot using functions like ode45, 
returns both the motor and the link side positions. 
Uses inverse dynamics control in the joint space.
   INPUTS: 
        t - output time vector [Nx1] - used by solver, i.e., ODE45
        Q - output state vector [(4*numJoints)x1] - used by solver, i.e.,
            ODE45
        td - desired time vector [Nx1] - used for linear interpolation of 
             positions
        Qd - input state vector (with accelerations) [Nx(3*numJoints)]
        robot - struct with robot properties
   OUTPUT: 
        Q_dot - joint velocity & acceleration vector [Nx(4*numJoints)] -
                used by solver, i.e., ODE45
%}

r = Q(1:robot.numJoints*2); % motor position and velocity
p = Q(robot.numJoints*2 + 1:end); % link position and velocity

% Position and velocity vectors
th = zeros(robot.numJoints,1); % motor-side variables
th_dot = zeros(robot.numJoints,1);

ql = zeros(robot.numJoints,1); % link-side variables
ql_dot = zeros(robot.numJoints,1);

pos_idx = 1:2:robot.numJoints*2;
vel_idx = 2:2:robot.numJoints*2;

th(:) = r(pos_idx); ql(:) = p(pos_idx);
th_dot(:) = r(vel_idx); ql_dot(:) = p(vel_idx);

% Inertia and Coriolis matrix
M = robot.M([ql(1),ql(2),ql(3)]);
% C = robot.C([ql_dot(1),ql_dot(2),ql_dot(3),ql(1),ql(2),ql(3)]);

% Structure of Qd
% Qd = [theta1,theta1_dot,theta1_ddot,theta2,theta2_dot,theta2_ddot,...]

% Commanded motor poisiton, velocity, and acceleration
theta1_in = interp1(td,Qd(:,1),t);
theta1_in_dot = interp1(td,Qd(:,2),t);
theta1_in_ddot = interp1(td,Qd(:,3),t);

theta2_in = interp1(td,Qd(:,4),t);
theta2_in_dot = interp1(td,Qd(:,5),t);
theta2_in_ddot = interp1(td,Qd(:,6),t);

theta3_in = interp1(td,Qd(:,7),t);
theta3_in_dot = interp1(td,Qd(:,8),t);
theta3_in_ddot = interp1(td,Qd(:,9),t);

% Controller gains
Kp = robot.jointController.Kp;
Kd = robot.jointController.Kd;

% Consolidate input values
theta_in = [theta1_in;theta2_in;theta3_in];
theta_in_dot = [theta1_in_dot;theta2_in_dot;theta3_in_dot];
theta_in_ddot = [theta1_in_ddot;theta2_in_ddot;theta3_in_ddot];


% Inverse dynamics acceleration
aq = theta_in_ddot + Kp.*(theta_in - th) + Kd.*(theta_in_dot - th_dot);

% Input torque to motor
Tm = M*aq + (robot.B - robot.jointDynamics.Bm)*ql_dot + robot.jointDynamics.Bm*th_dot; % inverse dynamics (without gravity compensation) % removed C*ql_dot

% Compute acceleration calculation
theta_ddot = robot.jointDynamics.Jm\(Tm - robot.jointDynamics.Bm*(th_dot - ql_dot) ...
    - robot.jointDynamics.Km*(th - ql)); % 3x1 motor acceleration vector

% Motor state vector derivative -- velocity and acceleration
theta_dot = zeros(robot.numJoints*2,1);

% Velocities
for i = vel_idx 
    theta_dot(i-1) = r(i);
end

% Accelerations
for k = 1:length(theta_ddot)
    theta_dot(2*k) = theta_ddot(k);
end

% Compute link acceleration
q_ddot = M\(-robot.B*ql_dot + robot.jointDynamics.Km*(th - ql)); % 3x1 link acceleration vector % removed -C*ql_dot 

% Link state vector derivative -- velocity and acceleration
q_dot = zeros(robot.numJoints*2,1);

% Velocities
for i = vel_idx 
    q_dot(i-1) = p(i);
end

% Accelerations
for k = 1:length(q_ddot)
    q_dot(2*k) = q_ddot(k);
end

% Store in the same vector
Q_dot = [theta_dot; q_dot];

end