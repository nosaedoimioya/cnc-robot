function [simTime,simInputs,simOutputs] = simulateUR5e(trajectory,robot,inTaskSpace,Ts,records)
%{
SIMULATEUR5E takes in a trajectory and simulates it on the UR5e robot
either in the task space control configuration or in the joint space
control configuration.

    INPUT(s):
        trajectory - [Nx3] trajectory points to send to the robot for task
                     space control [x,y,z], or [N x numJoints] trajectory
                     points to send to the robot for joint space control.
        robot - pre-defined struct of robot with its parameters and functions
        inTaskSpace - logical (bool) that represents whether the trajectory
                      is a task space trajectory or not
        Ts - sample time [s]
        records - [1 x k] array with strings representing the data that
                  is recorded.
    OUTPUT(s):
        simTime - [Nx1] vector of simulation time [s]
        simInputs - [Nxm] matrix of commanded states to be recorded and
                    returned
        simOutputs - [Nxk] matrix of output states to be recorded and 
                     returned
%}

% Check if records is given
if nargin < 5
    % Create the records array
    additionalDataColumns = 3; % for accelerations

    records = repmat("",1,robot.numJoints + additionalDataColumns);
    for i = 1:robot.numJoints
        records(i) = "q"+i;
    end

    records(i+1:end) = ["x_ddot","y_ddot","z_ddot"];
    % records = ["q1","q2","q3","q4","q5","q6","x_ddot","y_ddot","z_ddot"];
    disp("Data to record not given. Using default and recording: " + ...
        strjoin(records, ', '));
end

N = size(trajectory,1);
td = 0:Ts:Ts*(N-1)';

simOutputs = zeros(N,length(records));
if inTaskSpace
    % Set position, velocity, and acceleration for simulation from 
    % trajectory
    xd = trajectory(:,1); yd = trajectory(:,2); zd = trajectory(:,3);
    xd_dot = gradient(xd,Ts); 
    yd_dot = gradient(yd,Ts);
    zd_dot = gradient(zd,Ts);
    xd_ddot = gradient(xd_dot,Ts); 
    yd_ddot = gradient(yd_dot,Ts);
    zd_ddot = gradient(zd_dot,Ts);
    Xd = [xd,xd_dot,xd_ddot,yd,yd_dot,yd_ddot,zd,zd_dot,zd_ddot]; 
    
    % Use inverse kinematics to determine initial joint configutation 
    % [1 x numJoints]
    qd0 = robot.ik(xd(1),yd(1),zd(1));

    % Initial position/velocity of joints
    Q0 = zeros(size(qd0,2)*2,1);
    for i = 1:size(qd0,2)
        qi0 = qd0(i);
        qi0_dot = 0; % initial velocity is zero
        Q0(2*(i-1)+1:2*i) = [qi0;qi0_dot];
    end
    
    % initial position/velocity of x-, y-, and z-axis
    X0 = [xd(1);0;yd(1);0;zd(1);0];

    P0 = [X0; Q0; Q0]; % 2x Q0 for the motor and link states
    % Simulate
    [t,P] = ode45(@(t,P) ur5e_flexible_joint_tsp_inv_dynamics(t,P,td,Xd,robot),td,P0);

    %{
      P returns:
        task space position and velocities e.g., P(:,1:6)
        motor angular position and velocities e.g., for 6 joints, P(:,7:18)
        link side angular position and velocities e.g., for 6 joints, P(:,19:30)
    %}

    % Robot readouts for simulated system
    numCartesianStates = 6;
    
    readouts = repmat("",1,size(P,2));
    readouts(1:numCartesianStates) = ["x","x_dot","y","y_dot","z","z_dot"];
    % Example readouts vector
%     readouts = ["x","x_dot","y","y_dot","z","z_dot",...
%         "q1m","q1m_dot","q2m","q2m_dot","q3m","q3m_dot","q4m","q4m_dot",...
%         "q5m","q5m_dot","q6m","q6m_dot",...
%         "q1","q1_dot","q2","q2_dot","q3","q3_dot","q4","q4_dot",...
%         "q5","q5_dot","q6","q6_dot"];

    for i = numCartesianStates+1:numCartesianStates+robot.numJoints
        readouts(2*(i-numCartesianStates-1) + numCartesianStates+1 : ...
            2*(i-numCartesianStates) + numCartesianStates) = ...
        ["q"+(i-numCartesianStates)+"m","q"+(i-numCartesianStates)+"m_dot"];
    end
    numJointStates = robot.numJoints*2;
    numAfterMotorJoints = numCartesianStates + numJointStates;
    for i = numAfterMotorJoints+1:numAfterMotorJoints+robot.numJoints
        readouts(2*(i-numAfterMotorJoints-1) + numAfterMotorJoints+1 : ...
            2*(i-numAfterMotorJoints) + numAfterMotorJoints) = ...
        ["q"+(i-numAfterMotorJoints),"q"+(i-numAfterMotorJoints)+"_dot"];
    end

    % Acceleration readouts (computed)
    x_ddot = gradient(P(:,2),t);
    y_ddot = gradient(P(:,4),t);
    z_ddot = gradient(P(:,6),t);
    accelerations = [x_ddot,y_ddot,z_ddot];
    acc_readouts = ["x_ddot","y_ddot","z_ddot"];

    % Make sure simulation was successful
    if length(t) == length(td)
        % Record simulation states we wish to record
        for j = 1:length(records)
            if ismember(records(j),readouts)
                simOutputs(:,j) = P(:,readouts == records(j));
            elseif ismember(records(j),acc_readouts)
                simOutputs(:,j) = accelerations(:,acc_readouts == records(j));
            end
        end  
    else
        error("The simulation terminated early due to an error.")
    end

    % Record the input joint position commands
    Qd = robot.ik(xd,yd,zd); % [N x numJoints]
    simInputs = [trajectory,Qd];
    simTime = t;
else
    % Use values from trajectory to determine joint positions, velocities, 
    % and accelerations (trajectory: [N x numJoints])
    Qd = zeros(N,size(trajectory,2)*3);

    % Initial joint states
    Q0 = zeros(size(trajectory,2)*2,1); % 2x for position and velocity
    for i = 1:size(trajectory,2)
        qid = trajectory(:,i);
        qid_dot = gradient(qid,Ts);
        qid_ddot = gradient(qid_dot,Ts);
        Qd(:,3*(i-1)+1:3*i) = [qid,qid_dot,qid_ddot];
        Q0(2*(i-1)+1:2*i) = [qid(1);qid_dot(1)]; % initial joint position and velocity
    end
    
    Q0 = [Q0; Q0]; % 2x Q0 for the motor and link states
    % Simulate
    [t,Q] = ode45(@(t,Q) ur5e_flexible_joint_inv_dynamics(t,Q,td,Qd,robot),td,Q0);

    %{
      Q returns:
        motor angular position and velocities e.g., for 6 joints, Q(:,1:12)
        link side angular position and velocities e.g., for 6 joints, Q(:,13:24)
    %}

    % Robot readouts for simulated system
    readouts = repmat("",1,size(Q,2));
    % Example readouts vector
%     readouts = ["q1m","q1m_dot","q2m","q2m_dot","q3m","q3m_dot","q4m","q4m_dot",...
%         "q5m","q5m_dot","q6m","q6m_dot",...
%         "q1","q1_dot","q2","q2_dot","q3","q3_dot","q4","q4_dot",...
%         "q5","q5_dot","q6","q6_dot"];
    for i = 1:robot.numJoints
        readouts(2*(i-1)+1 : 2*i) = ["q"+i+"m","q"+i+"m_dot"];
    end
    numAfterMotorJoints = robot.numJoints*2;
    for i = numAfterMotorJoints+1:numAfterMotorJoints+robot.numJoints
        readouts(2*(i-numAfterMotorJoints-1) + numAfterMotorJoints+1 : ...
            2*(i-numAfterMotorJoints) + numAfterMotorJoints) = ...
        ["q"+(i-numAfterMotorJoints),"q"+(i-numAfterMotorJoints)+"_dot"];
    end

    % Acceleration readouts (computed using forward kinematics)
    % Get the link angular positions
    Q_link = Q(:,robot.numJoints*2+1:end);
    Qp_link = Q_link(:,1:2:end); % just the position commanded
    
    X = robot.fk(Qp_link);
    x_ddot = gradient(gradient(X(:,1),t),t);
    y_ddot = gradient(gradient(X(:,2),t),t);
    z_ddot = gradient(gradient(X(:,3),t),t);
    accelerations = [x_ddot,y_ddot,z_ddot];
    acc_readouts = ["x_ddot","y_ddot","z_ddot"];

    % Make sure simulation was successful
    if length(t) == length(td)
        % Record simulation states we wish to record
        for j = 1:length(records)
            if ismember(records(j),readouts)
                simOutputs(:,j) = Q(:,readouts == records(j));
            elseif ismember(records(j),acc_readouts)
                simOutputs(:,j) = accelerations(:,acc_readouts == records(j));
            end
        end  
    else
        error("The simulation terminated early due to an error.")
    end

    % Record the input joint position commands
    Xd = robot.fk(trajectory);
    simInputs = [Xd,trajectory];
    simTime = t;
end

end