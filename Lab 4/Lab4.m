clc
clearvars -except udpObj
close all

%% Qn 4.1 and 4.2
% DH parameters calculated
DH = [ 0 400 25 pi/2 ; 0 0 315 0 ; 0 0 35 pi/2 ; 0 365 0 -pi/2 ; 0 0 0 pi/2 ; 0 161.44 -156 0 ];
kuka = mykuka(DH);

% Create DH table with a6=0 for better results when deriving the attactive
% and repulsive force functions
DHForces = [ 0 400 25 pi/2 ; 0 0 315 0 ; 0 0 35 pi/2 ; 0 365 0 -pi/2 ; 0 0 0 pi/2 ; 0 161.44 0 0 ];
kukaForces=mykuka(DHForces);

% Set z_grid initially to 45mm. May tweak value
z_grid=55;

% Define points
P0 = [370; -440; 150];
P1 = [370; -440; z_grid]; % z_grid = 45mm
P2 = [750; -220; 225];
P3 = [620; 350; 225];

% Instantiate 3 obstacles
setupobstacle;

% Set gripper orientation
R = [0 0 1 ; 0 -1 0 ; 1 0 0]; 

% Compute homogeneous matrices at each defined endpt
Hhome=forwardkuka([0 pi/2 0 0 pi/2 0],kuka);
H0=[R P0;zeros(1,3) 1];
H1=[R P1;zeros(1,3) 1];
H2=[R P2;zeros(1,3) 1];
H3=[R P3;zeros(1,3) 1];

% Compute joint angles when the gripper is oriented at each endpt
qHome=inversekuka(Hhome,kuka); 
q0 = inversekuka( H0,kuka);
q1 = inversekuka( H1,kuka);
q2 = inversekuka( H2,kuka);
q3 = inversekuka( H3,kuka);
forwardkuka(q0',kuka)

% Transition q6 from initial to final value
t = linspace(0,10,300);

% Perform motion planning from HOME to p0, p0 to p1, p1 to p2, p2 to p3
% (ensuring that the gripper is pointing towards the ground at each endpt
tolerance=0.03;
qrefHome0=motionplan(qHome, q0,0,10, kukaForces, [],tolerance);
qrefHome0=ppval(qrefHome0,t)';
qref01=motionplan(q0, q1,0,10, kukaForces, [],tolerance);
qref01=ppval(qref01,t)';
qref12=motionplan(q1, q2,0,10, kukaForces, obs,tolerance);
qref12=ppval(qref12,t)';
qref23=motionplan(q2, q3,0,10, kukaForces, obs,tolerance);
qref23=ppval(qref23,t)';

%Here is where we tested some of the positions of the obstacles
startingMarjer1=[370; -440; 80];
endingMarker=[750; -220; 100];
startingMarker2=[620; 350; 100];


HstartingMarker1=[R startingMarjer1;zeros(1,3) 1];
HEndingMarker=[R endingMarker;zeros(1,3) 1];
HstratingMarker2=[R startingMarker2;zeros(1,3) 1];

qstratingMarker2 = inversekuka( HstartingMarker1,kuka);
qEndingMarker = inversekuka( HEndingMarker,kuka);
qstratingMarker2 = inversekuka( HstratingMarker2,kuka);



%Here is where we use the motionplan algorithm we created before 
% to find points along the motion. Then we use a 
%third order spline to connect these points so a path is created
qref01=motionplan(q0, q1,0,10, kukaForces, [],tolerance);
qref01=ppval(qref01,t)';
qref12=motionplan(q1, q2,0,10, kukaForces, obs,tolerance);
qref12=ppval(qref12,t)';
qref23=motionplan(q2, q3,0,10, kukaForces, obs,tolerance);
qref23=ppval(qref23,t)';


% start of controlling the robot, first set home and open gripper
setHome(0.04)
setGripper(0)

%make it go to the location above the cube
setAngles(qstratingMarker2,0.04)
setGripper(1)

%since it is just a vertical downwards, the artificial potential field
%proves bad since it makes the robot curve to the side, therefore knocking
%the cube over. Therefore, a simple linear downward motion is performed.
setAngles(qref01(1,:),0.04)
setAngles(qref01(end,:),0.04)
setGripper(1)
%robot moves from where it first grabbed the cube, to the cup
for eachPOint = 1:length(qref12)
    setAngles(qref12(eachPOint,:),0.04)
end

for eachPOint = 1:length(qref23)
    setAngles(qref23(eachPOint,:),0.04)
end
%robot drops cube into cup
setGripper(0)




%Comments of varying of parameters.
%{
As expected, if we vary the attrative alpha, the forces that are applied to
the motors are increased, therefore the robot moved faster towards the goal
The repulsion force created a stronger repulsive field, so there is a wider
gap between the minimum distance between the robot and the obstacle.
Varying rho changes the range of influence, so that previously, the robot
was only attracted to the object while in the increased rho case, the
robots initial movments seemed to be a combination of avioding obstacles as
well as reaching the final goal. An interesting phenonmenon occurs when
these parameters are varied at the same time. if the aplha and eta are
approximately the same, the robot would do kind of like a sawtooth pattern
between being more attracted to the goal, and also being repulsed by the 
obstacle. 

}%