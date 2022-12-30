%% Question 3.1
% DH parameters calculated for my puma robot in Lab 1
DH = [ 0 76 0 pi/2 ; 0 -23.65 43.23 0 ; 0 0 0 pi/2 ; 0 43.18 0 -pi/2 ; 0 0 0 pi/2 ; 0 20 0 0 ];

% Instantiate robot with above defined DH parameters using mypuma560 function
myrobot = mypuma560(DH);

% Test for att.m
% Output should be [ -0.9050 -0.0229 -0.4003 -0.0977 0.1034 0.0000 ]
H1 = eul2tr([0 pi pi/2]);
H1(1:3,4) = 100 * [-1; 3; 3;] / 4;
q1 = inverse(H1, myrobot);
H2 = eul2tr([0 pi -pi/2]);
H2(1:3,4) = 100 * [3; -1; 2;] / 4;
q2 = inverse(H2,myrobot);
tau = att(q1,q2,myrobot)

%% Question 3.2
% Plot the trajectory of the robot arm. Puma 560 should converge to desired
% final position
qref = motionplan(q1,q2,0,10,myrobot,[],0.01);
t=linspace(0,10,300);
q = ppval(qref,t)';
plot(myrobot,q)

%% Question 3.3
% Test for rep.m with cylinder obstacle
% Output should be [ 0.9950    0.0291   -0.0504    0.0790    0.0197    0.0000 ]
setupobstacle
q3 = 0.9 * q1 + 0.1 * q2;
tau = rep(q3,myrobot,obs{1})

% Test for rep.m with sphere obstacle
% Output should be [ -0.1138   -0.2140   -0.9702    0.0000   -0.0037    0.0000 ]
q = [pi/2 pi 1.2*pi 0 0 0];
tau = rep(q,myrobot,obs{6})

% Plot robot arm with obstacles
setupobstacle
hold on
axis([-100 100 -100 100 0 200])
view(-32,50)
plotobstacle(obs);
qref = motionplan(q1,q2,0,10,myrobot,obs,0.01);
t = linspace(0,10,300);
q = ppval(qref,t)';
plot(myrobot,q);
hold off
