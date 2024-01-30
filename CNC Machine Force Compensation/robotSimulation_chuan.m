% This code simulates a flexible-joint robot through a trajectory
% Written by: Nosa Edoimioya
% Date: 1/29/2024

close all
clear all
clc

%% Define sampling frequency/time and other parameters
fs = 250; % [Hz]
Ts = 1/fs; % [s]

% Set the parameters of the system ID bang-coast-bang trajectory
% =========================================
% *** output_sensor := ['ToolAcc', 'JointPos'] ***

% Joint-space example
trajParams = struct('configuration', 'joint', ...
                    'output_sensor', 'ToolAcc', ...
                    'max_velocity', 18, ... % [rad/s]
                    'max_displacement', pi/18, ... % [rad]
                    'max_acceleration', 50, ... % [rad/s^2]
                    'single_pt_run_time', 4); % [s]

% Set the zero/home position of the robot
% =========================================
% Should be [1 x numJoints] for joint
% space control.
zero_pos = [0,0,0]; % for joint space control of ThreeLinkUr5e
% zero_pos = [0,0,0,0,0,0]; % for joint space control of UR5e

%% Run trajectory simulation
% Get robot data
robot = loadRobotData("ThreeLinkUr5e");
% robot = loadRobotData("UR5e");

% Generate trajectory
move_axis = randi([1,length(zero_pos)],1,1); % Choose a random axis to test
[trajectory,~] = generateSinglePointTrajectory(robot,trajParams,zero_pos,move_axis,Ts);

% Run simulation and save data
% =========================================
% Check that the starting position of the robot matches the configuration
inTaskSpace = strcmp(trajParams.configuration,'task');
if inTaskSpace
    if ~(length(zero_pos) == 3)
        error("Configuration " + trajParams.configuration + " requires " + ...
            "starting position vector of length 3. Input vector has " + ...
            "length " + length(start_pos) + ".")
    end
elseif ~(length(zero_pos) == robot.numJoints) % configuration is 'joint'
    error("Configuration " + trajParams.configuration + " requires " + ...
        "starting position vector of length " + robot.numJoints + ". " + ...
        "Input vector has length " + length(start_pos) + ".")
end

% Get data structure for robot
dataStructureFunction = "getDataStructure" + robot.name; % e.g., getDataStructureUR5e

getDataStructure = str2func(dataStructureFunction);
robotData.records = getDataStructure();

% Run trajectory on robot
simRobotFunction = 'simulate' + robot.name; % e.g., simulateUR5e
runRobot = str2func(simRobotFunction);

[t,inputStates,outputStates] = runRobot(trajectory,robot,inTaskSpace,Ts,robotData.records);

% Save data (including desired trajectory)
robotData.outputData = outputStates;
robotData.inputData = inputStates;
robotData.time = t;

%{ 
robotData - struct with the following properties
            records: [1 x k] array with strings representing the data that
                     is recorded.
            outputData: [N x k] matrix with data from the robot
                k := ["q1","q2","q3","x_ddot","y_ddot","z_ddot"];
            inputData: [N x m] matrix with position and joint commands to the robot
                axesToCommand is the number of commanded axes
                m := ["x_des","y_des","z_des","q1_des","q2_des","q3_des"]
            time: [N x 1] vector with time data
             recorded from the robot.
%}

%% Plotting

% Desired joint/Cartesian position vs. output
if inTaskSpace
    % Run forward kinematics to get x,y,z
    [x,y,z] = robot.fk(outputStates(:,1:robot.numJoints));
    X = [x,y,z];

    plot(t,inputStates(:,3+move_axis))
    hold on
    plot(t,X(:,move_axis)*1000)
    xlabel("Time")
    ylabel("Position [mm]")
    legend("Desired", "Output")
else
    plot(t,inputStates(:,3+move_axis))
    hold on
    plot(t,outputStates(:,move_axis))
    xlabel("Time")
    ylabel("Position [rad]")
    legend("Desired", "Output")
end

%% Prepare square trajectory
load("square_vector_v150_a5mps2.mat")
xd = xTBI;
yd = yTBI;
t_d = 0:Ts:(length(xd)-1)*Ts';
x_d = xTBI/1000;
y_d = yTBI/1000;
z_d = zeros(size(x_d));

% Offset of x-axis to make sure it doesn't stretch past robot reach
xOffset = 0.04; 

% The desired trajectory is in the end effector - need
% to do a transformation from the end effector frame to base frame.
P_ee = [x_d.'+ xOffset;y_d.';z_d.';ones(1,length(x_d))];

% Get the transformation matrix of the robot at the zero position
% using forward kinematics
T_0E = eye(4); % [4x4] identity matrix
for k = 1:robot.numJoints
    % Robot transformation matrix from commanded joint to the end
    % joint. Each iteration multiplies the next joint's
    % transformation matrix, given its (initial) position
    T_0E = T_0E * robot.T(k,zero_pos.',1,robot.a,robot.d, ...
        robot.alpha);
end

% Transform position from frame 6 to frame 0, starting at initial position
x_d0 = zeros(size(x_d));
y_d0 = zeros(size(x_d));
z_d0 = zeros(size(x_d));
for i = 1:length(x_d)
    p_0E = T_0E*P_ee(:,i);
    x_d0(i,1) = p_0E(1); y_d0(i,1) = p_0E(2); z_d0(i,1) = p_0E(3);
end

figure
plot3(x_d*1000,y_d*1000,z_d*1000,'LineWidth',2)
axis equal
xlabel('X [mm]')
ylabel('Y [mm]')
zlabel('Z [mm]')
title('Original')

figure
plot3(x_d0*1000,y_d0*1000,z_d0*1000,'LineWidth',2)
axis equal
xlabel('X [mm]')
ylabel('Y [mm]')
zlabel('Z [mm]')
title('Transformed (view from base frame)')
%% Execute square trajectory
% Compute joint positions with inverse kinematics
qTrajectorySquare = robot.ik(x_d0, y_d0, z_d0);

[t,inputStates,outputStates] = runRobot(qTrajectorySquare,robot,inTaskSpace,Ts,robotData.records);

% Plot joint positions
figure
hold on
legendStrs = [];
for i = 1:robot.numJoints
    plot(t,inputStates(:,3+i),'LineWidth',1)
    legendStrs = [legendStrs; "qd_" + num2str(i) ];
end
for i = 1:robot.numJoints
    plot(t,outputStates(:,i),'--','LineWidth',1)
    legendStrs = [legendStrs; "q_" + num2str(i) ];
end
ylabel('Joint Position [rad]')
xlabel('Time [s]')
title('Joint Position')
legend(legendStrs)
