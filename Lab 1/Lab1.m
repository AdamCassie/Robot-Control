%% Question 4.1
% DH parameters calculated (changed from prelab)
DH = [ 0 76 0 pi/2 ; 0 -23.65 43.23 0 ; 0 0 0 pi/2 ; 0 43.18 0 -pi/2 ; 0 0 0 pi/2 ; 0 20 0 0 ];

% Instantiate robot with above defined DH parameters using mypuma560 function
myrobot = mypuma560(DH);


%% Question 4.2
% Generate sequence of 200 joint angles
q = zeros(200, 6);
q(:, 1) = linspace(0, pi, 200).';
q(:, 2) = linspace(0, pi / 2, 200).';
q(:, 3) = linspace(0, pi, 200).';
q(:, 4) = linspace(pi / 4, (3 * pi) / 4, 200).';
q(:, 5) = linspace(-pi / 3, pi / 3, 200).';
q(:, 6) = linspace(0, 2 * pi, 200).';
q 
% % Plot robot evolution corresponding to joint angles generated above
% figure('Name', 'Evolution through joint angles generated')
% plot(myrobot, q)
%  
% 
% %% Question 4.3
% % Test for forward.m
% % End effector traces red trajectory
% o = zeros(200, 3);
% for i = 1:200
%     H = forward(q(i, :), myrobot);
%     o(i, :) = H(1:3, 4); 
% end
% figure('Name', 'Evolution through rototranslation computed in FKP')
% plot3(o(:, 1), o(:, 2), o(:, 3), 'r')
% hold on
% plot(myrobot, q)
% 
% 
% %% Question 4.4
% % Test for inverse.m
% % Output should be [ -0.0331 -1.0667 1.0283 3.1416 3.1032 0.8185 ]
% H_d = [ cos(pi/4) -sin(pi/4) 0 20 ; sin(pi/4) cos(pi/4) 0 23 ; 0 0 1 15 ; 0 0 0 1 ];
% q = inverse(H_d, myrobot);
% q
% 
% %% Question 4.1.1
% % Define sequence of desired end effector positions
% d(:, 1) = linspace(10,30, 100).';
% d(:, 2) = linspace(23,30, 100).';
% d(:, 3) = linspace(15,100, 100).';
% 
% %% Question 4.4.2
% % Define Rotation Matrix for desired constant orientation of end effector
% R_d = [ cos(pi/4) -sin(pi/4) 0 ; sin(pi/4) cos(pi/4) 0 ; 0 0 1 ];
% 
% %% Question 4.4.3
% % Solve Inverse Kinematics problem for every position defined in d
% q = zeros(100, 6);
% for i = 1:100
%     % Set the homogeneous transformation matrix for the position defined by
%     % this row of d
%     H_d = zeros(4, 4);
%     H_d(1:3, 1:3) = R_d;
%     H_d(1:3, 4) = d(i, :);
%     H_d(4, 4) = 1;
%     
%     % Compute the joint angles for the position defined by this row of d
%     q(i, :) = inverse(H_d, myrobot);
%     
% end
% 
% %% Question 4.4.4
% % Plot robot evolution corresponding to joint angles computed above
% figure('Name', 'Evolution through joint angles computed in IKP')
% plot3(d(:, 1), d(:, 2), d(:, 3), 'r')
% hold on
% plot(myrobot, q)







