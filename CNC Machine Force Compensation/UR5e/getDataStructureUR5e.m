function dataArray = getDataStructureUR5e()
%{
GETDATASTRUCTUREUR5E This simple function defines the type of data that we
can/need to collect from the robot. It returns an array of strings with
descriptions of the data that will be collected from the machine. 

The strings follow a certain format: 'axis' or 'axis_{dot | ddot}'. For
example, q1 represents the angular position of joint 1, x represents the
Cartesian x-position of the end-effector, q3_dot represents the angular
velocity of joint 3 and y_ddot represents the Cartesian y-acceleration of
the end-effector.
    INPUT(s):
        
    OUTPUT(s):
        dataArray - [1 x N] array of strings representing data to be 
                    retrieved from the robot. Follows a specific string
                    format.
%}
dataArray = ["q1","q2","q3","q4","q5","q6","x_ddot","y_ddot","z_ddot"];

end