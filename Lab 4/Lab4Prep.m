clear all

% Code related to the prep component of Lab 4

% DH parameters calculated
DH = [ 0 400 25 pi/2 ; 0 0 315 0 ; 0 0 35 pi/2 ; 0 365 0 -pi/2 ; 0 0 0 pi/2 ; 0 161.44 -156 0 ];

% Instantiate robot with above defined DH parameters using mykuka function
kuka = mykuka(DH);

% Create DH table with a6=0 for better results when deriving the attactive
% and repulsive force functions
DHForces = [ 0 400 25 pi/2 ; 0 0 315 0 ; 0 0 35 pi/2 ; 0 365 0 -pi/2 ; 0 0 0 pi/2 ; 0 161.44 0 0 ];
kukaForces=mykuka(DHForces);

% Test for repulsive function
setupobstacle_lab4prep;
obs = prepobs;
tau = rep([pi/10,pi/12,pi/6,pi/2,pi/2,-pi/6],kukaForces,prepobs{1});

% Test for motion planning algorithm
p1 = [620 375 50];
p2 = [620 -375 50];
R = [0 0 1 ; 0 -1 0 ; 1 0 0];
H1=[R p1';zeros(1,3) 1];
H2=[R p2';zeros(1,3) 1];
q1 = inversekuka( H1,kuka);
q2 = inversekuka( H2,kuka);
tolerance=0.03;
qref=motionplan(q1, q2,0,10, kukaForces, obs,tolerance);

t=linspace(0,10,300);
q=ppval(qref,t)';
plot(kuka,q);
hold off

obs1_coord=[620;0;100]
H_obs1=[R obs1_coord';zeros(1,3) 1];
qobs = inversekuka( H_obs1,kuka);
qrefObs=motionplan(q1, qobs,0,10, kukaForces, obs,tolerance);