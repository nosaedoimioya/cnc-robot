function [UR5e] = getUR5e()
%{
GETUR5E returns a Matlab struct of important robot parameters and functions

   OUTPUT(s): 
        UR5e - struct of the robot with the following properties
            name: name of the robot - string
            numJoints: number of joints of the robot - integer
            Jm_vec: Inertia vector of the motors at the joints -
                    [1 x numJoints] array
            Bm_vec: Damping constant vector of the motors at the joints -
                    [1 x numJoints] array
            Km_vec: Stiffness constant vector of the motors at the joints - 
                    [1 x numJoints] array
            Jm: joint inertia matrix - [numJoints x numJoints] diagonal 
                matrix
            Bm: joint damping matrix - [numJoints x numJoints] diagonal 
                matrix
            Km: joint stiffness matrix - [numJoints x numJoints] diagonal 
                matrix
            Kp: joint proportional constants vector - [1 x numJoints] array
            Kd: joint derivative constants vector - [1 x numJoints] array
            M: inertia matrix function - returns [numJoints x numJoints] matrix
               input(s): the six joint position vector ([q1,q2,q3,q4,q5,q6])
            C: Coriolis matrix function - returns [numJoints x numJoints] matrix
                    input(s): the six joint velocities and positions
                    (dq1,dq2,dq3,dq4,dq5,dq6,q1,q2,q3,q4,q5,q6)
            Cd: Christoffel diagonal matrix function - computes the elements to be 
                multiplied by the squared joint velocities - returns [numJoints x 
                numJoints] matrix
                    input(s): the six joint positions (q1,q2,q3,q4,q5,q6)
            Cod: Christoffel off-diagonal matrix function - computes the elements
                to be multiplied by the non-squared joint velocities - returns
                [numJoints x numJoints*(numJoints-1)/2] matrix
                    input(s): the six joint positions (q1,q2,q3,q4,q5,q6)
            B: constant link damping matrix as a [numJoints x numJoints] matrix
            a: link lengths [m] - [1 x numJoints] array (D-H parameter)
            alpha: link twist [rad] - [1 x numJoints] array (D-H parameter)
            d: link offset [m] - [1 x numJoints] array (D-H parameter)
            T: function to generate tranformation matrix at the current joint
               configuration from joint n-1 to joint n. Returns homogeneous [4x4]
               matrix.
                    input(s):  n - joint number (row of theta)
                               theta - matrix of joint positions [numJoints 
                                       x numCols]; theta(n,c) defines the 
                                       joint position we are interested in. 
                                       There can be 1 column or several 
                                       columns stored in the matrix.
                               c - joint column (column of theta)
                               a - [1 x numJoints] 'a' vector from the D-H 
                                   parameters [m]
                               d - [1 x numJoints] 'd' vector from the D-H 
                                   parameters [m]
                               alpha - [1 x numJoints] 'alpha'vector from 
                                       the D-H parameters [rad] 
            ik: inverse kinematics function - returns a [N x numJoints] matrix
                of the robot joint positions, where N is the number of time
                steps/positions computed for
                    input(s): x, [N x 1] vector of x-axis positions
                              y, [N x 1] vector of y-axis positions
                              z, [N x 1] vector of z-axis positions
                              frame, string to specify 'end-effector' or 'base' frame
                              for which the x,y,z positions are defined.
                              (end-effector positions defined relative to current
                              position as a zero postion)
            fk: forward kinematics function - returns a [N x 3] matrix of the x
            (1), y (2) and z (3) axis positions of the end-effector, where N is the
            number of time steps/positions computed for
                input(s): the joint positions [q1,q2,q3,q4,q5,q6] as a [N x
                          numJoints] matrix
            J: Jacobian matrix function - returns a [6 x numJoints] matrix, 
            representing the Jacobian matrix of the robot, which is used to compute 
            the Cartesian velocity and angular velocity of the end-effector when 
            multiplied by the velocities of the robot's joints
                input(s): q1, [1x1] joint 1 position
                          q2, [1x1] joint 2 position
                          ...
                          q{numJoints}, [1x1] joint {numJoints} position
            Jdot: Jacobian time derivative function - returns a [6 x numJoints] 
            matrix, representing derivative of the Jacobian matrix of the robot, 
            which is useful to compute the Cartesian acceleration and angular 
            acceleration of the end-effector.
                inputs(s): dq1, [1x1] joint 1 velocity
                           ...
                           dq{numJoints}, [1x1] joint numJoints velocity
                           q1, [1x1] joint 1 position
                           ...
                           q{numJoints}, [1x1] joint {numJoints} position
            
%}

% Constants
hz2rps = 2*pi; % Hz to rad/s

%{ 
Predefine motor structs:
    wn: motor natural frequency [rad/s]
    zeta: motor damping ratio [0-1] 
    Jm: motor rotor inertia [Nm-s^2/rad]
    wc: PD controller frequency [rad/s]
    zetac: PD controller damping ratio [0-1] 
%}

motor1 = struct('wn', 180*hz2rps, 'zeta', 0.7, 'Jm', 320e-4, ...
                'wc', 190*hz2rps, 'zetac', 1);

motor2 = struct('wn', 260*hz2rps, 'zeta', 0.95, 'Jm', 80e-4, ...
                'wc', 265*hz2rps, 'zetac', 1);


%{ 
Predefined robots structs:
    name: robot name
    numJoints: number of joints
    
%}

% ------------- UR5E Definition --------------
UR5e.name = "UR5e";
UR5e.numJoints = 6;

% Inertia vector of the motors at the joints ([1 x numJoints] array)
UR5e.Jm_vec = [ones(1,UR5e.numJoints/2) * motor1.Jm, ... 
               ones(1,UR5e.numJoints/2) * motor2.Jm];

% Damping constant vector of the motors at the joints ([1 x numJoints] array)
UR5e.Bm_vec = [2*ones(1,UR5e.numJoints/2) * motor1.zeta * motor1.wn * motor1.Jm, ... 
               2*ones(1,UR5e.numJoints/2) * motor2.zeta * motor2.wn * motor2.Jm];

% Stiffness constant vector of the motors at the joints ([1 x numJoints] array)
UR5e.Km_vec = [ones(1,UR5e.numJoints/2) * motor1.wn^2 * motor1.Jm, ... 
               ones(1,UR5e.numJoints/2) * motor2.wn^2 * motor2.Jm];

%{ 
Matrices of motor joint dynamics
    Jm: joint inertia matrix ([numJoints x numJoints] diagonal matrix)
    Bm: joint damping matrix ([numJoints x numJoints] diagonal matrix)
    Km: joint stiffness matrix ([numJoints x numJoints] diagonal matrix) 
%}
UR5e.jointDynamics = struct('Jm', diag(UR5e.Jm_vec), ...
                            'Bm', diag(UR5e.Bm_vec), ...
                            'Km', diag(UR5e.Km_vec));

% Joint controller natural frequency vector ([1 x numJoints] array)
UR5e.wc_vec = [ones(1,UR5e.numJoints/2) * motor1.wc, ... 
               ones(1,UR5e.numJoints/2) * motor2.wc];

% Joint controller damping ratio vector ([1 x numJoints] array)
UR5e.zetac_vec = [ones(1,UR5e.numJoints/2) * motor1.zetac, ... 
               ones(1,UR5e.numJoints/2) * motor2.zetac];

%{ 
Vectors of motor joint PD controllers
    Kp: joint proportional constants ([1 x numJoints] array)
    Kd: joint derivative constants ([1 x numJoints] array)
%}
UR5e.jointController = struct('Kp',((UR5e.wc_vec.^2 .* UR5e.Jm_vec) - UR5e.Km_vec),...
                              'Kd',( (2*UR5e.zetac_vec .* UR5e.wc_vec .* ...
                              UR5e.Jm_vec) - UR5e.Bm_vec ) );

%{
Link dynamics
    M: inertia matrix function - returns [numJoints x numJoints] matrix
       input(s): the six joint position vector ([q1,q2,q3,q4,q5,q6])
    C: Coriolis matrix function - returns [numJoints x numJoints] matrix
       input(s): the six joint velocities and positions
       (dq1,dq2,dq3,dq4,dq5,dq6,q1,q2,q3,q4,q5,q6)
    Cd: Christoffel diagonal matrix function - computes the elements to be 
    multiplied by the squared joint velocities - returns [numJoints x 
    numJoints] matrix
       input(s): the six joint positions (q1,q2,q3,q4,q5,q6)
    Cod: Christoffel off-diagonal matrix function - computes the elements
    to be multiplied by the non-squared joint velocities - returns
    [numJoints x numJoints*(numJoints-1)/2] matrix;
    B: constant link damping matrix as a [numJoints x numJoints] matrix
%}
UR5e.M = @ur5e_inertia_matrix;
UR5e.C = @ur5e_coriolis_matrix;
UR5e.Cd = @ur5e_christoffel_diagonals;
UR5e.Cod = @ur5e_christoffel_off_diagonals;
UR5e.B = UR5e.jointDynamics.Bm * (3/2);

%{
Denavit-Hartenburg Parameters
    a: link lengths [m] - [1 x numJoints] array
    alpha: link twist [rad] - [1 x numJoints] array
    d: link offset [m] - [1 x numJoints] array
%}
UR5e.a = [0,-0.425,-0.3922,0,0,0]; % [m]
UR5e.alpha = [pi/2,0,0,pi/2,-pi/2,0]; % [rad]
UR5e.d = [0.1625,0,0,0.1333,0.0997,0.0996]; % [m]

%{
Inverse and Forward Kinematics
    ik: inverse kinematics function - returns a [N x numJoints] matrix
    of the robot joint positions, where N is the number of time
    steps/positions computed for
        input(s): x, [N x 1] vector of x-axis positions
                  y, [N x 1] vector of y-axis positions
                  z, [N x 1] vector of z-axis positions
                  frame, string to specify 'end-effector' or 'base' frame
                  for which the x,y,z positions are defined.
                  (end-effector positions defined relative to current
                  position as a zero postion)
    fk: forward kinematics function - returns a [N x 3] matrix of the x
    (1), y (2) and z (3) axis positions of the end-effector, where N is the
    number of time steps/positions computed for
        input(s): the joint positions [q1,q2,q3,q4,q5,q6] as a [N x
        numJoints] matrix
%}
UR5e.ik = @ur5e_inv_kinematics;
UR5e.fk = @ur5e_fwd_kinematics;

%{
General transformation matrix
    T: function to generate tranformation matrix at the current joint
    configuration from joint n-1 to joint n. Returns homogeneous [4x4]
    matrix.
        input(s):  n - joint number (row of theta)
                   theta - matrix of joint positions [numJoints x numCols];
                           theta(n,c) defines the joint position we are 
                           interested in. There can be 1 column or several 
                           columns stored in the matrix.
                   c - joint column (column of theta)
                   a - [1 x numJoints] 'a' vector from the D-H parameters [m]
                   d - [1 x numJoints] 'd' vector from the D-H parameters [m]
                   alpha - [1 x numJoints] 'alpha'vector from the D-H 
                           parameters [rad] 

%}
UR5e.T = @ur5e_transformation_matrix;

%{
    J: Jacobian matrix function - returns a [6 x numJoints] matrix, 
    representing the Jacobian matrix of the robot, which is used to compute 
    the Cartesian velocity and angular velocity of the end-effector when 
    multiplied by the velocities of the robot's joints
        input(s): q1, [1x1] joint 1 position
                  q2, [1x1] joint 2 position
                  ...
                  q{numJoints-1}, [1x1] joint {numJoints-1} position
    Jdot: Jacobian time derivative function - returns a [6 x numJoints] 
    matrix, representing derivative of the Jacobian matrix of the robot, 
    which is useful to compute the Cartesian acceleration and angular 
    acceleration of the end-effector.
        inputs(s): dq1, [1x1] joint 1 velocity
                   ...
                   dq{numJoints-1}, [1x1] joint numJoints velocity
                   q1, [1x1] joint 1 position
                   ...
                   q{numJoints-1}, [1x1] joint {numJoints-1} position
%}
UR5e.J = @ur5e_jacobian;
UR5e.Jdot = @ur5e_jacobian_derivative;

% ------------- End of UR5E Definition --------------

end