function [ThreeLinkUr5e] = getThreeLinkUr5e()
%{
GETUR5E returns a Matlab struct of important robot parameters and functions

   OUTPUT(s): 
        ThreeLinkUr5e - struct of the robot with the following properties
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
ThreeLinkUr5e.name = "ThreeLinkUr5e";
ThreeLinkUr5e.numJoints = 3;

% Inertia vector of the motors at the joints ([1 x numJoints] array)
ThreeLinkUr5e.Jm_vec = ones(1,ThreeLinkUr5e.numJoints) * motor1.Jm;

% Damping constant vector of the motors at the joints ([1 x numJoints] array)
ThreeLinkUr5e.Bm_vec = 2*ones(1,ThreeLinkUr5e.numJoints) * motor1.zeta * motor1.wn * motor1.Jm;

% Stiffness constant vector of the motors at the joints ([1 x numJoints] array)
ThreeLinkUr5e.Km_vec = ones(1,ThreeLinkUr5e.numJoints) * motor1.wn^2 * motor1.Jm;

%{ 
Matrices of motor joint dynamics
    Jm: joint inertia matrix ([numJoints x numJoints] diagonal matrix)
    Bm: joint damping matrix ([numJoints x numJoints] diagonal matrix)
    Km: joint stiffness matrix ([numJoints x numJoints] diagonal matrix) 
%}
ThreeLinkUr5e.jointDynamics = struct('Jm', diag(ThreeLinkUr5e.Jm_vec), ...
                            'Bm', diag(ThreeLinkUr5e.Bm_vec), ...
                            'Km', diag(ThreeLinkUr5e.Km_vec));

% Joint controller natural frequency vector ([1 x numJoints] array)
ThreeLinkUr5e.wc_vec = ones(1,ThreeLinkUr5e.numJoints) * motor1.wc;

% Joint controller damping ratio vector ([1 x numJoints] array)
ThreeLinkUr5e.zetac_vec = ones(1,ThreeLinkUr5e.numJoints) * motor1.zetac;

%{ 
Vectors of motor joint PD controllers
    Kp: joint proportional constants ([1 x numJoints] array)
    Kd: joint derivative constants ([1 x numJoints] array)
%}
ThreeLinkUr5e.jointController = struct('Kp',((ThreeLinkUr5e.wc_vec.^2 .* ThreeLinkUr5e.Jm_vec) - ThreeLinkUr5e.Km_vec),...
                              'Kd',( (2*ThreeLinkUr5e.zetac_vec .* ThreeLinkUr5e.wc_vec .* ...
                              ThreeLinkUr5e.Jm_vec) - ThreeLinkUr5e.Bm_vec ) );

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
ThreeLinkUr5e.M = @threeLinkUr5e_inertia_matrix;
ThreeLinkUr5e.C = @threeLinkUr5e_coriolis_matrix;
ThreeLinkUr5e.Cd = @threeLinkUr5e_christoffel_diagonals;
ThreeLinkUr5e.Cod = @threeLinkUr5e_christoffel_off_diagonals;
ThreeLinkUr5e.B = ThreeLinkUr5e.jointDynamics.Bm * (3/2);

%{
Denavit-Hartenburg Parameters
    a: link lengths [m] - [1 x numJoints] array
    alpha: link twist [rad] - [1 x numJoints] array
    d: link offset [m] - [1 x numJoints] array
%}
ThreeLinkUr5e.a = [0,-0.425,-0.3922,0]; % [m]
ThreeLinkUr5e.alpha = [pi/2,0,0,0]; % [rad]
ThreeLinkUr5e.d = [0.1625,0,0,0.1333]; % [m]

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
ThreeLinkUr5e.ik = @threeLinkUr5e_inv_kinematics;
ThreeLinkUr5e.fk = @threeLinkUr5e_fwd_kinematics;

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
ThreeLinkUr5e.T = @threeLinkUr5e_transformation_matrix;

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
ThreeLinkUr5e.J = @threeLinkUr5e_jacobian;
ThreeLinkUr5e.Jdot = @threeLinkUr5e_jacobian_derivative;

% ------------- End of UR5E Definition --------------

end