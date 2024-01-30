function [Q_dot] = ur5e_flexible_joint_inv_dynamics(t,Q,td,Qd,robot)
%{
UR5E_FLEXIBLE_JOINT_INV_DYNAMICS This function simulates the closed loop 
flexible body dynamics of a 6 DOF UR5e robot using functions like ode45, 
returns both the motor and the link side positions. 
Uses inverse dynamics control in the joint space.
   INPUTS: 
        t - output time vector [Nx1] - used by solver, i.e., ODE45
        Q - output state vector [Nx(4*numJoints)] - used by solver, i.e.,
            ODE45
        td - desired time vector [Nx1] - used for linear interpolation of 
             positions
        Qd - input state vector (with accelerations) [Nx(3*numJoints)]
        robot - struct with robot properties
   OUTPUT: 
        Q_dot - joint velocity & acceleration vector [Nx(4*numJoints)] -
                used by solver, i.e., ODE45
%}

r = Q(1:12); % motor position and velocity
p = Q(13:end); % link position and velocity

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

M = robot.M([q1,q2,q3,q4,q5,q6]);
% C = robot.C(q1_dot,q2_dot,q3_dot,q4_dot,q5_dot,q6_dot,q1,q2,q3,q4,q5,q6);

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

theta4_in = interp1(td,Qd(:,10),t);
theta4_in_dot = interp1(td,Qd(:,11),t);
theta4_in_ddot = interp1(td,Qd(:,12),t);

theta5_in = interp1(td,Qd(:,13),t);
theta5_in_dot = interp1(td,Qd(:,14),t);
theta5_in_ddot = interp1(td,Qd(:,15),t);

theta6_in = interp1(td,Qd(:,16),t);
theta6_in_dot = interp1(td,Qd(:,17),t);
theta6_in_ddot = interp1(td,Qd(:,18),t);

% Controller gains
Kp = robot.jointController.Kp;
Kd = robot.jointController.Kd;

% Consolidate input values
theta_in = [theta1_in;theta2_in;theta3_in;theta4_in;theta5_in;theta6_in];
theta_in_dot = [theta1_in_dot;theta2_in_dot;theta3_in_dot;...
                theta4_in_dot;theta5_in_dot;theta6_in_dot];
theta_in_ddot = [theta1_in_ddot;theta2_in_ddot;theta3_in_ddot;...
                 theta4_in_ddot;theta5_in_ddot;theta6_in_ddot];

% Position and velocity vectors
th_dot = [th1_dot;th2_dot;th3_dot;th4_dot;th5_dot;th6_dot]; % motor velocity vector
ql_dot = [q1_dot;q2_dot;q3_dot;q4_dot;q5_dot;q6_dot]; % link velocity vector

th = [th1;th2;th3;th4;th5;th6]; % motor position vector
ql = [q1;q2;q3;q4;q5;q6]; % link position vector


% Inverse dynamics acceleration
aq = theta_in_ddot + Kp.*(theta_in - th) + Kd.*(theta_in_dot - th_dot);

% Input torque to motor
Tm = M*aq + (robot.B - robot.jointDynamics.Bm)*ql_dot + robot.jointDynamics.Bm*th_dot; % inverse dynamics (without gravity compensation) % removed C*ql_dot

% Motor acceleration calculation
theta_ddot = robot.jointDynamics.Jm\(Tm - robot.jointDynamics.Bm*(th_dot - ql_dot) ...
    - robot.jointDynamics.Km*(th - ql)); % 6x1 motor acceleration vector

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
q_ddot = M\(-robot.B*ql_dot + robot.jointDynamics.Km*(th - ql)); % 6x1 link acceleration vector % removed -C*ql_dot 

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

end