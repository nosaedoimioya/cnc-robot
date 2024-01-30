function [trajectory,endIndices] = generateSinglePointTrajectory(robot,trajParams,start_position,move_axis,Ts)
%{
GENERATESINGLEPOINTTRAJECTORY takes in the robot, trajectory parameters,
the current position of the robot and the system sample time, and returns 
a random bang-coast-bang trajectory that will be sent to the robot to get
the dynamics at a single point
   INPUT(s):
        robot - pre-defined struct of robot with its parameters and functions
        trajParams - parameters for the system ID trajectory including:
            configuration: the control configuration of the robot, 'joint'
                or 'task' space (default) configuration - string
            max_displacement: the maximum displacement of the robot in
                the given configuration - double [m]
            max_velocity: the maximum velocity of the robot in the given
                configuration - double [m/s]
            max_acceleration: the maximum acceleration of the robot in the
                given configuration - double [m/s^2]
            single_pt_run_time: the amount of time for each sequence of
                bang-coast-bang signals at each point
        start_position - starting position of the robot - given as [1x3] vector
            when the configuration is 'task' and as [1xnumJoints] vector 
            when the configuration is 'joint'.
        move_axis - axis of the robot to move with the current
                    bang-coast-bang signal
        Ts - sample time in seconds
        
   OUTPUTS(s):
        trajectory - [Nx3] trajectory points to send to the robot for 'task'
                     space control [x,y,z], or [N x numJoints] trajectory
                     points to send to the robot for 'joint' space control.
        endIndices - cell that stores the vectors of the end index for each
                     bang-coast-bang signal. Useful for signal processing
                     later. {3x1} cell for 'task' config and {numJoints x
                     1} cell for 'joint' config.
        
%}

% Set predefined list of valid configurations // for DEBUGGING
predefinedConfigs = ["joint", "task"];

% Check that the configuration is given // for DEBUGGING
if ~isfield(trajParams,'configuration')
    trajParams.configuration = 'task'; 
    disp("Control configuration is not defined. Default: ''task'' being " + ...
        "used.")
elseif ~isstring(trajParams.configuration) && ~ischar(trajParams.configuration)
    error(trajParams.configuration + " is not a valid string. Please enter a" + ...
        " string representing the control configuration of the robot." + ...
        " You can enter: %s", strjoin(predefinedConfigs, ', '))
elseif ~ismember(trajParams.configuration, predefinedConfigs)
    error("The configuration " + trajParams.configuration + " is not a " + ...
        "valid control configuration. " + ...
        "Enter a configuration name from the predefined list: %s", ...
        strjoin(predefinedConfigs, ', '))
end

% Check that the trajectory parameters are defined. Define defaults if not
% defined
if ~isfield(trajParams,'max_displacement')
    trajParams.max_displacement = 0.010; % [m]
    disp("Maximum displacement not defined. Default (0.010 m) for task " + ...
        "space control is being used. Redefine to use joint space " + ...
        "control as desired.")
elseif ~isnumeric(trajParams.max_displacement)
    error("The max_displacement entered is not a valid number. Please enter a" + ...
        " number representing the maximum dispacement of the robot in meters.")
end

if ~isfield(trajParams,'max_velocity')
    trajParams.max_velocity = 0.400; % [m/s]
    disp("Maximum velocity not defined. Default (0.400 m/s) for task " + ...
        "space control is being used. Redefine to use joint space " + ...
        "control as desired.")
elseif ~isnumeric(trajParams.max_velocity)
    error("The max_velocity entered is not a valid number. Please enter a" + ...
        " number representing the maximum velocity of the robot in m/s.")
end

if ~isfield(trajParams,'max_acceleration')
    trajParams.max_acceleration = 100; % [m/s^2]
    disp("Maximum acceleration not defined. Default (100 m/s^2) for task " + ...
        "space control is being used. Redefine to use joint space " + ...
        "control as desired.")
elseif ~isnumeric(trajParams.max_acceleration)
    error("The max_acceleration entered is not a valid number. Please enter a" + ...
        " number representing the maximum acceleration of the robot in m/s^2.")
end

if ~isfield(trajParams,'single_pt_run_time')
    trajParams.single_pt_run_time = 4; % [s]
    disp("Single point run time is not defined. Default (4 s) is being " + ...
        "used.")
elseif ~isnumeric(trajParams.single_pt_run_time)
    error("The single_pt_run_time entered is not a valid number. Please enter a" + ...
        " number representing the maximum acceleration of the robot in seconds.")
end

% Check that the starting position of the robot matches the configuration
if strcmp(trajParams.configuration,'task')
    if ~(length(start_position) == 3)
        error("Configuration '" + trajParams.configuration + "' requires " + ...
            "starting position vector of length 3. Input vector has " + ...
            "length " + length(start_position) + ".")
    end
elseif ~(length(start_position) == robot.numJoints) % configuration is 'joint'
    error("Configuration '" + trajParams.configuration + "' requires " + ...
        "starting position vector of length " + robot.numJoints + ". " + ...
        "Input vector has length " + length(start_position) + ".")
end

% Check that the move axis of the robot matches the configuration
if strcmp(trajParams.configuration,'task')
    if ~(move_axis >= 1 && move_axis <= 3)
        error("Configuration '" + trajParams.configuration + "' requires " + ...
            "a move axis between 1 and 3 (inclusive). Move axis given " + ...
            "is " + move_axis + ".")
    end
elseif ~(move_axis >= 1 && move_axis <= robot.numJoints) % configuration is 'joint'
    error("Configuration '" + trajParams.configuration + "' requires " + ...
        "a move axis between 1 and " + robot.numJoints + " (inclusive). " + ...
        "Move axis given is " + move_axis + ".")
end

% Define trajectory parameters
params.dq_max = trajParams.max_displacement;
params.dq_min = -trajParams.max_displacement;

params.v_max = trajParams.max_velocity;
params.v_min = trajParams.max_velocity / 4; % 25% of the max

params.a_max = trajParams.max_acceleration;
params.a_min = 0;

% Max and min waiting time
params.t_wait_max = trajParams.single_pt_run_time / 4; % dwell up to 25% of run time
params.t_wait_min = params.t_wait_max / 10; 

% Start and end velocity of trajectory
params.vs = 0;
params.ve = 0;

% Time that bang-coast-bang must be completed by
params.end_time = trajParams.single_pt_run_time - 2*params.t_wait_max;

% Bang-coast-bang plus dwell time
params.extend_time = trajParams.single_pt_run_time;

% Generate the trajectory
k = length(start_position);
T = 0:Ts:params.extend_time;
N = length(T);

trajectory = zeros(N,k);
endIndices = cell(1,1);

% Get bang-coast-bang motion for the move axis
[t_p,perturbation,endIdx] = random_bang_coast_bang(params,Ts,start_position(move_axis));

% Fill perturbation with period of no motion
perturbation_ext = [perturbation;
            ones(round((params.extend_time-t_p(end))/Ts),1)*perturbation(end)];

% Check if the perturbation time is larger than given time
if (length(perturbation_ext) == N)
    trajectory(:,move_axis) = perturbation_ext;
else
    error("The given trajectory parameters produced a system ID trajectory " + ...
        "that takes longer than the specified time. Please adjust " + ...
        "the trajectory parameters. You can start by decreasing the " + ...
        "max_displacement or increasing the max_velocity.")
end

% Hold the other positions still
for j = 1:k
    if move_axis ~= j
        hold_position = ones(size(perturbation_ext))*start_position(j);
        trajectory(:,j) = hold_position;
    end
end

% Store end indices
endIndices{1,1} = endIdx;
