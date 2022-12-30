%% Question 3
% DH parameters calculated
DH = [ 0 400 25 pi/2 ; 0 0 315 0 ; 0 0 35 pi/2 ; 0 365 0 -pi/2 ; 0 0 0 pi/2 ; 0 161.44 -296.23 0 ];

% Instantiate robot with above defined DH parameters using mykuka function
kuka = mykuka(DH);

% Test for forward_kuka.m
ans = forward_kuka([pi/5 pi/3 -pi/4 pi/4 pi/3 pi/4]', kuka);
ans

% Test for inverse_kuka.m
inverse_kuka(ans, kuka)

%% Question 4.3
% Read joint angles of robot at sample point
q = getAngles();

% Find optimal delta to minimize the total joint variable error
delta = fminunc(@deltajoint, [0 0]);

% Redefine robot structure using updated DH parameters
kuka = mykuka_search(delta);

% Test calibration by solving inverse kinematics problem
setHome(0.04);
R6to0 = [ 0 0 1 ; 0 -1 0 ; 1 0 0 ];
O6to0 = [547.1900 -236.5100   81.4000]; % Set this to be X1 sample point
H = zeros(4, 4);
H(1:3, 1:3) = R6to0;
H(1:3, 4) = O6to0;
H(4, 4) = 1;
q = inverse_kuka(H, kuka);
setAngles(q, 0.04);

%% Question 4.4
% Define target point in Workspace coordinates
p_workspace = [ 600 ; 100 ; 10 ];

% Convert point in base frame coordinates
p_baseframe = FrameTransformation(p_workspace);

% Choose desired position of end effector with respect to base frame
R = [ 0 0 1 ; 0 -1 0 ; 1 0 0 ];

% Define the desired homogeneous transformation matrix of the end effector
% with respect to frame 0
H = [ R p_baseframe ;  zeros(1, 3) 1 ];

% Use inverse kinematics to find the desired joint variables
q = inverse_kuka(H, kuka);

% Command robot to target position
setAngles(q, 0.04);

%% Question 4.5
% Draw line segment
mysegment(kuka);

% Draw circle
mycircle(kuka);

% Draw creative pattern
myellipse(kuka);

% Draw the given jug pattern
% myjug(kuka);
