clc
clearvars -except udpObj
close all

%% Qn 4.3

% setup of the dh table and roboot configuration
%DH = [ 0 400 25 pi/2 ; 0 0 315 0 ; 0 0 35 pi/2 ; 0 365 0 -pi/2 ; 0 0 0 pi/2 ; 0 161.44 -296.23 0 ];
DH = [ 0 400 25 pi/2 ; 0 0 315 0 ; 0 0 35 pi/2 ; 0 365 0 -pi/2 ; 0 0 0 pi/2 ; 0 161.44 -156 0 ];
kuka = mykuka(DH);

DHForces = [ 0 400 25 pi/2 ; 0 0 315 0 ; 0 0 35 pi/2 ; 0 365 0 -pi/2 ; 0 0 0 pi/2 ; 0 161.44 0 0 ];
kukaForces=mykuka(DHForces);





%define positions that robot needs to be
z_grid=55;

P0 = [370; -440; 150];
P1Cube1 = [370; -440; 100];
P1Cube2 = [370; -440; 67]; 
P1Cube3 = [370; -440; 50]; 

P3 = [620; 350; 225];

pobs1=[620;0;750];
pobs2=[620;-440;750];

setupobstacle;



%get the Homogeneous matrix of the positions
R = [0 0 1 ; 0 -1 0 ; 1 0 0];
Hhome=forwardkuka([0 pi/2 0 0 pi/2 0],kuka);
H0=[R P0;zeros(1,3) 1];
HP1Cube1=[R P1Cube1;zeros(1,3) 1];
HP1Cube2=[R P1Cube2;zeros(1,3) 1];
HP1Cube3=[R P1Cube3;zeros(1,3) 1];
HP3=[R P3;zeros(1,3) 1];
Hpobs1=[R pobs1;zeros(1,3) 1];
Hpobs2=[R pobs2;zeros(1,3) 1];

%find the joint abgles based on those homogeneous matrices
qHome=inversekuka(Hhome,kuka);
q0 = inversekuka( H0,kuka);
qHP1Cube1 = inversekuka( HP1Cube1,kuka);
qHP1Cube2 = inversekuka( HP1Cube2,kuka);
qHP1Cube3 = inversekuka( HP1Cube3,kuka);
qHP3 = inversekuka( HP3,kuka);
qHpobs1 = inversekuka( Hpobs1,kuka);
qHpobs2 = inversekuka( Hpobs2,kuka);



t = linspace(0,10,300);
tolearance=0.03;




%initially there is two cubes stacked. Go to the first cube and grab it
disp("Starting the stream")

setHome(0.04)
setGripper(0)
setAngles(q0,0.04)
%setAngles(qHpobs1,0.04)
setGripper(1)
setGripper(0)
setAngles(qHP1Cube1,0.04)
setGripper(1)



%go to the cylinder, and then drop it inside


qrefCub1Cyl1=motionplan(qHP1Cube1, qHpobs1,0,10, kukaForces, obs,tolearance);
qrefCub1Cyl1=ppval(qrefCub1Cyl1,t)';
for eachPOint = 1:length(qrefCub1Cyl1)
    setAngles(qrefCub1Cyl1(eachPOint,:),0.04)
end
setGripper(0)




%go back to the cubes, and grab it
qrefCyl1Q0=motionplan(qHpobs1, q0,0,10, kukaForces, obs,tolearance);
qrefCyl1Q0=ppval(qrefCyl1Q0,t)';

for eachPOint = 1:length(qrefCyl1Q0)
    setAngles(qrefCyl1Q0(eachPOint,:),0.04)
end

setAngles(qHP1Cube2,0.04)
setGripper(1)



%go to the cup and drop it in

P1 = [370; -440; z_grid]; % z_grid = 45mm
P2 = [750; -220; 225];
P3 = [620; 350; 225];

H1=[R P1;zeros(1,3) 1];
H2=[R P2;zeros(1,3) 1];
H3=[R P3;zeros(1,3) 1];

q1 = inversekuka( H1,kuka);
q2 = inversekuka( H2,kuka);
q3 = inversekuka( H3,kuka);


qrefHome0=motionplan(qHome, q0,0,10, kukaForces, [],tolearance);
qrefHome0=ppval(qrefHome0,t)';
qref01=motionplan(q0, q1,0,10, kukaForces, [],tolearance);
qref01=ppval(qref01,t)';
qref12=motionplan(q1, q2,0,10, kukaForces, obs,tolearance);
qref12=ppval(qref12,t)';
qref23=motionplan(q2, q3,0,10, kukaForces, obs,tolearance);
qref23=ppval(qref23,t)';

for eachPOint = 1:length(qref12)
    setAngles(qref12(eachPOint,:),0.04)
end

for eachPOint = 1:length(qref23)
    setAngles(qref23(eachPOint,:),0.04)
end
setGripper(0)