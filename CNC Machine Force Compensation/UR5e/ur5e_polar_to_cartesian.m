function [x,y,z] = ur5e_polar_to_cartesian(v,r,robot,q1)
%{
UR5E_POLAR_TO_CARTESIAN takes in the polar position (v, r), the robot
struct, and the base joint position, and returns the x-, y-, and z-axis
position of the robot.
   INPUT(s):
        v - positive angle from the horizontal [rad]
        r - positive radius from robot base [m]
        robot - pre-defined struct of robot with its parameters and functions
        q1 - angular position of the robot base [rad]
   OUTPUT(s):
        x - x-axis end-effector position in the workspace with respect to
            the base frame [m]
        y - y-axis end-effector position in the workspace with respect to
            the base frame [m]
        z - z-axis end-effector position in the workspace with respect to
            the base frame [m]
%}
x = -r*cos(v)*cos(q1);
y = -(robot.d(4) + robot.d(6)) - r*cos(v)*sin(q1);
z = robot.d(1) + r*sin(v);
end