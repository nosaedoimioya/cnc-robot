function [V,R,polarToCartesian] = getPolarCoordinatesUR5e(numOfPositions,robot)
%{
GETPOLARCOORDINATESUR5E takes in a struct with the number of V and R
positions that are desired (nV and nR properties, respectively) and
the robot struct, and returns a vector of V and R positions as well as a
function, polar to cartesian to convert task space polar coordinates to 
the Cartesian coordinates

    INPUT(s):
        numOfPositions - # of positions struct including:
            nV: the number of angles from the horizontal to check - int
            nR: the number of polar radii to check - int
        robot - pre-defined struct of robot with its parameters and functions

    OUTPUT(s):
        V - [1 x numOfPositions.nV] array of angular positions [rad]
        R - [1 x numOfPositions.nR] array of radii [m]
        polarToCartesian - function pointer that returns the Cartesian
            position of a robot, given the polar coordinates
                input(s):
                    v: angle from the horizontal [rad]
                    r: radius from robot base [m]
%}

% Check if the numberOfPositions has the required fields
if ~isfield(numOfPositions,'nV') || ~isfield(numOfPositions,'nR')
    error("System ID parameters does not contain one or more of the " + ...
        "required properties: nV, number of angles to traverse, nR, number " + ...
        "of radii to traverse.")
end

% Check if the numberOfPositions are integers
if (~(isnumeric(numOfPositions.nV) && (round(numOfPositions.nV) == numOfPositions.nV)) ...
     || ~(isnumeric(numOfPositions.nR) && (round(numOfPositions.nR) == numOfPositions.nR)))
    error("One or more of the system ID parameters are not in the requested " + ...
        "integer (rounded double) format: nV: (" + ...
        class(numOfPositions.nV) + ") "+ numOfPositions.nV + ...
        " and nR: (" + class(numOfPositions.nR) + ") "+ numOfPositions.nR + ".")
end

% Set V to span 90 deg from the horizontal (at nV locations)
nV = numOfPositions.nV;
V = linspace(0,pi/2,nV);

% set R to span 30 to 80% of the full length of the robot's stretched arm
nR = numOfPositions.nR;
L = sum(abs(robot.a)); % sum of robot link lengths as given in a (the D-H parameter)
minR = 0.3*L; maxR = 0.8*L;
R = linspace(minR,maxR,nR);

% Set polarToCartesian function
polarToCartesian = @ur5e_polar_to_cartesian;
end