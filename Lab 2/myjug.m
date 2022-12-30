function myjug(myrobot)
    % Generate end effector positions
    data = xlsread('jug.xlsx');
    xdata = 550 + 10 * data(:, 1);
    ydata= 10 * data(:, 2);
    zdata= -ones(length(data), 1) * 6;
    X_workspace= [xdata, ydata, zdata]';
    
    % Convert columns of X_workspace to coordinates of frame 0
    X_baseframe = zeros(3, 100);
    for i = 1 : 100
        X_baseframe(:, i) = FrameTransformation(X_workspace(:, i));
    end
    theAverate=mean(X_baseframe(3,:));
    X_baseframe(3,:)=theAverate*ones(1,100);
    
    % Compute and set joint angles for each column in X_baseframe
    R6to0 = [ 0 0 1 ; 0 -1 0 ; 1 0 0 ];
    for i = 1 : 100
       H = [ R6to0 X_baseframe(:, i) ;  zeros(1, 3) 1 ]; 
       q = inverse_kuka(H, myrobot);
       setAngles(q, 0.04);
    end

end