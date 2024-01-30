function [P_dot] = threeLinkUr5e_flexible_joint_tsp_inv_dynamics(t,P,td,Xd,robot)
%{
THREELINKUR5E_FLEXIBLE_JOINT_TSP_INV_DYNAMICS This function simulates the closed loop 
flexible body dynamics of a 6 DOF UR5e robot using functions like ode45, 
returns both the motor and the link side positions. 
Uses inverse dynamics control in the task space.
   INPUTS: 
        t - output time vector [Nx1] - used by solver, i.e., ODE45
        P - output state vector [(2*numCartesianStates + 4*numJoints)x1] - 
            used by solver, i.e., ODE45
        td - desired time vector [Nx1] - used for linear interpolation of 
             positions
        Xd - input Cartesian state vector (with accelerations)
             [Nx(3*numCartesianStates)]
        robot - struct with robot properties
   OUTPUT: 
        P_dot - [Xdot, Qdot], task and joint space position & velocity 
                vector [(2*numCartesianStates + 4*numJoints)x1]
                Xdot, task position & velocity vector
                [(2*numCartesianStates)x1]
                Qdot, joint (motor & link) position & velocity vector
                [(4*numJoints)x1]
        ODE45 returns a [Nx(2*numCartesianStates + 4*numJoints)] matrix
        collecting the P_dot from eact time step in td
%}

X = P(1:6); % Cartesian end-effector position and velocities
Q = P(7:end); % Motor and link position and velocities

% x, y, and z position / velocity
x = X(1); x_dot = X(2);
y = X(3); y_dot = X(4);
z = X(5); z_dot = X(6);

% Get the joint positions for computing dynamics
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

% Get mass matrix
M = robot.M([ql(1),ql(2),ql(3)]);
% C = robot.C(ql_dot(1),ql_dot(2),ql_dot(3),ql(1),ql(2),ql(3));

% Controller gains
Kp = robot.jointController.Kp.';
Kd = robot.jointController.Kd.';

% Commanded poisiton, velocity, and acceleration
x_in = interp1(td,Xd(:,1),t);
x_dot_in = interp1(td,Xd(:,2),t);
x_ddot_in = interp1(td,Xd(:,3),t);

y_in = interp1(td,Xd(:,4),t);
y_dot_in = interp1(td,Xd(:,5),t);
y_ddot_in = interp1(td,Xd(:,6),t);

z_in = interp1(td,Xd(:,7),t);
z_dot_in = interp1(td,Xd(:,8),t);
z_ddot_in = interp1(td,Xd(:,9),t);

% Consolidate input and output values
X_ddot_in = [x_ddot_in;y_ddot_in;z_ddot_in];
X_dot_in = [x_dot_in;y_dot_in;z_dot_in];
X_in = [x_in;y_in;z_in];
Xo = [x;y;z];
Xo_dot = [x_dot;y_dot;z_dot];

% Inverse dynamics acceleration
aX = X_ddot_in + Kp.*(X_in - Xo) + Kd.*(X_dot_in - Xo_dot);

% Get robot jacobian and jacobian derivative
J = robot.J(th);
J_dot = robot.Jdot(th_dot,th);

% Compute motor inverse dynamics acceleration
aq = pinv(J)*(aX - J_dot*th_dot);

% Input torque to motor
Tm = M*aq + (robot.B - robot.jointDynamics.Bm)*ql_dot + robot.jointDynamics.Bm*th_dot; % inverse dynamics (removed C*ql_dot)

% Motor acceleration calculation
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

% Link acceleration
q_ddot = M\(-robot.B*ql_dot + robot.jointDynamics.Km*(th - ql)); % removed -C*ql_dot

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


Q_dot = [theta_dot; q_dot];

% Compute end-effector acceleration
X_ddot = J*q_ddot + J_dot*ql_dot;

% End-effector velocity and acceleration
X_dot = zeros(6,1);
% Velocities
for i = 2:2:6 
    X_dot(i-1) = X(i);
end

% Accelerations
for k = 1:length(X_ddot)
    X_dot(2*k) = X_ddot(k);
end

X_dot(1) = X(2);
X_dot(2) = X_ddot(1);
X_dot(3) = X(4);
X_dot(4) = X_ddot(2);
X_dot(5) = X(6);
X_dot(6) = X_ddot(3);

P_dot = [X_dot; Q_dot];

end