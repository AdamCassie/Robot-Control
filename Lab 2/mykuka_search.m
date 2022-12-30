% Function to define a new robot structure with the KUKA parameters
% Arguments for mykuka function: 
% DH = a 6 * 4 matrix of DH parameters when the robot is in generic position

function myrobot = mykuka_search(delta)
    
    DH = [ 0 400 25 pi/2 ; 0 0 315 0 ; 0 0 35 pi/2 ; 0 365 0 -pi/2 ; 0 0 0 pi/2 ; 0 (161.44 + delta(2)) -(296.23 + delta(1)) 0 ];
    
    % Construct link objects using DH table parameters
    for i = 1:6
            links(i) = Link(DH(i,:), 'standard');
    end
    
    % Construct a serial-link arm-type robot to represent the KUKA using
    % the links generated above
    myrobot = SerialLink(links, 'name', 'mykuka');
    
end
