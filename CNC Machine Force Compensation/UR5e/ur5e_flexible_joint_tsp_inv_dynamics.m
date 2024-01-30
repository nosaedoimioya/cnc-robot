function [P_dot] = ur5e_flexible_joint_tsp_inv_dynamics(t,P,td,Xd,robot)
%{
UR5E_FLEXIBLE_JOINT_TSP_INV_DYNAMICS This function simulates the closed loop 
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
r = Q(1:12);
p = Q(13:end);

th1 = r(1); th1_dot = r(2);
th2 = r(3); th2_dot = r(4);
th3 = r(5); th3_dot = r(6);
th4 = r(7); th4_dot = r(8);
th5 = r(9); th5_dot = r(10);
th6 = r(11); th6_dot = r(12);

q1 = p(1); q1_dot = p(2);
q2 = p(3); q2_dot = p(4);
q3 = p(5); q3_dot = p(6);
q4 = p(7); q4_dot = p(8);
q5 = p(9); q5_dot = p(10);
q6 = p(11); q6_dot = p(12);

% Get mass matrix
M = robot.M([q1,q2,q3,q4,q5,q6]);

% Controller gains
Kp = robot.jointController.Kp;
Kd = robot.jointController.Kd;

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

% Position and velocity vectors
th_dot = [th1_dot;th2_dot;th3_dot;th4_dot;th5_dot;th6_dot]; % motor velocity vector
ql_dot = [q1_dot;q2_dot;q3_dot;q4_dot;q5_dot;q6_dot]; % link velocity vector

theta = [th1;th2;th3;th4;th5;th6]; % motor position vector
ql = [q1;q2;q3;q4;q5;q6]; % link position vector

% Get robot jacobian and jacobian derivative
J = robot.J(theta);
J_dot = robot.Jdot(th_dot,theta);

% Compute motor inverse dynamics acceleration
aq = J\(aX - J_dot*th_dot);

% Input torque to motor
Tm = M*aq + (robot.B - robot.jointDynamics.Bm)*ql_dot + robot.jointDynamics.Bm*th_dot; % inverse dynamics (removed C*ql_dot)

% Motor acceleration calculation
theta_ddot = robot.jointDynamics.Jm\(Tm - robot.jointDynamics.Bm*(th_dot - ql_dot) ...
    - robot.jointDynamics.Km*(theta - ql)); % 6x1 motor acceleration vector

% Motor state vector derivative -- velocity and acceleration
theta_dot = zeros(12,1);
theta_dot(1) = r(2);
theta_dot(2) = theta_ddot(1);
theta_dot(3) = r(4);
theta_dot(4) = theta_ddot(2);
theta_dot(5) = r(6);
theta_dot(6) = theta_ddot(3);
theta_dot(7) = r(8);
theta_dot(8) = theta_ddot(4);
theta_dot(9) = r(10);
theta_dot(10) = theta_ddot(5);
theta_dot(11) = r(12);
theta_dot(12) = theta_ddot(6);

% Link acceleration
q_ddot = M\(-robot.B*ql_dot + robot.jointDynamics.Km*(theta - ql)); % removed -C*ql_dot

% Link state vector derivative -- velocity and acceleration
q_dot = zeros(12,1);
q_dot(1) = p(2);
q_dot(2) = q_ddot(1);
q_dot(3) = p(4);
q_dot(4) = q_ddot(2);
q_dot(5) = p(6);
q_dot(6) = q_ddot(3);
q_dot(7) = p(8);
q_dot(8) = q_ddot(4);
q_dot(9) = p(10);
q_dot(10) = q_ddot(5);
q_dot(11) = p(12);
q_dot(12) = q_ddot(6);

Q_dot = [theta_dot; q_dot];

X_ddot = J*q_ddot + J_dot*ql_dot;

X_dot = zeros(6,1);
X_dot(1) = X(2);
X_dot(2) = X_ddot(1);
X_dot(3) = X(4);
X_dot(4) = X_ddot(2);
X_dot(5) = X(6);
X_dot(6) = X_ddot(3);

P_dot = [X_dot;Q_dot];

end