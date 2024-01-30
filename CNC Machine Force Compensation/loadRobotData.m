function [robot] = loadRobotData(robotName)
%{
LOADROBOTDATA takes the name of the robot as an input and returns a Matlab
struct that contains constant parameters and functions used to define 
the behavior of the robot.

   INPUT(s): 
       robotName - name of the robot (string); must be in the list of 
                   strings below for which the robot behavior is
                   predifined.
   OUTPUT(s):
       robot - Matlab struct of the robot
%}

if ~isstring(robotName) && ~ischar(robotName)
    error("Input " + robotName + " is not a valid string or character array.")
end

% Predefined list of robots
predefinedRobots = ["UR5e", "ThreeLinkUr5e"];

% Check if the input robot is in the predefined list of robots
robotIsValid = ismember(robotName, predefinedRobots);
if ~robotIsValid
    error("The robot " + robotName + " is not a valid robot. " + ...
        "Enter a robot name on the predefined list: %s", ...
        strjoin(predefinedRobots, ', '))
end

functionName = 'get' + robotName; % e.g., 'getUR5e'

% Execute function to get robot struct
getRobot = str2func(functionName);
robot = getRobot();

end