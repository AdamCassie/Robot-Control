% Function to draw a 50mm radius circle centred at [620 0 -1]

function mycircle(myrobot)
    % Generate end effector positions
    X_workspace = zeros(3, 100);
    X_workspace(1, :) = 50 * cos(linspace(0, 2 * pi, 100)) + 620;
    X_workspace(2, :) = 50 * sin(linspace(0, 2 * pi, 100));
    X_workspace(3, :) = linspace(-8, -8, 100);
    
    % Convert columns of X_workspace to coordinates of frame 0
    X_baseframe = zeros(3, 100);
    for i = 1 : 100
        X_baseframe(:, i) = FrameTransformation(X_workspace(:, i));
    end

    % Compute and set joint angles for each column in X_baseframe
    R6to0 = [ 0 0 1 ; 0 -1 0 ; 1 0 0 ];
    for i = 1 : 100
       H = [ R6to0 X_baseframe(:, i) ;  zeros(1, 3) 1 ]; 
       q = inverse_kuka(H, myrobot);
       setAngles(q, 0.04);
    end
    
    
end