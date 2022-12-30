% Function to define a new robot structure with the PUMA 560 parameters
% Arguments for mypuma560 function: 
% DH = a 6 * 4 matrix of DH parameters when the robot is in rest position

function myrobot = mypuma560(DH)
    
    % Construct link objects using DH table parameters
    for i = 1:6
            links(i) = Link(DH(i,:), 'standard');
    end
    
    % Construct a serial-link arm-type robot to represent the PUMA560 using
    % the links generated above
    myrobot = SerialLink(links, 'name', 'mypuma560');
    
end
