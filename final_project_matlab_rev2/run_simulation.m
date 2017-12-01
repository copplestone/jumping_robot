% clear

% We can organize our code by filing things in different folders.  These
% folders need to be added to the Matlab path so that it can run the files
% inside them even when they are not the current folder listed at the top
% of the Matlab window.  For more information about the current folder, see
% http://www.mathworks.com/help/matlab/matlab_env/understanding-file-locations-in-matlab.html
% For more information about the Matlab path, see
% http://www.mathworks.com/help/matlab/matlab_env/what-is-the-matlab-search-path.html
setpath                                     % add AutoDerived, Modeling, and Visualization folders to Matlab path

p = parameters();                           % get parameters from file
%z0 = [0; 1; 0 ;0];                    % set initial state [x,y,th1,th2]
%z0 = [0;1.327;pi/2.05;-0.05*pi;0;0.0;0;0];
z0 = [0;0.415; pi/2.01;-0.03*pi;0;0.0;0;0];
%% 

% An equation has been added to dynamics_continuous and dynamics_discrete
% to integrate this new state.

% set guess
%x = [1.8, 1.8,0,0,1.4,0.2,-0.5]
%x = [1.8, 1.8,0.6,-0.2,0.005,-0.005,0.0]

groundTorques = [0.6,-0.2,0.3,-0.005,0.0];
airAngles = [0,0,-pi/1.2,0,0]; 
x = [0.7, 0.7, groundTorques, airAngles]
tf = x(1);                                        % simulation final time
ctrl.tf = x(2);                                  % control time points
%ctrl.T = [1.5 1.5 1.5];                               % control values
ctrl.T = [x(3) x(4) x(5) x(6) x(7)];
ctrl.Ang = airAngles;

%from first successful
%  ctrl.tf = 0.7396;
%  ctrl.T = [0.5624 -0.4773 1.3667 -0.3349 -0.2812];
%  ctrl.Ang = [0 0 -2.6180 0 0];
% 
%   ctrl.tf = 0.6053;
%  ctrl.T = [0.5430 -0.2937 1.3921 0.2361 -0.2707];
%  ctrl.Ang = [0 0 -2.6180 0 0];
%  
%  ctrl.tf = 0.6001 
%  ctrl.T =[0.3597 0.3880 1.2977 -0.7291 -0.0996]
%  ctrl.Ang=[0 0 -2.6180 0 0]
 
 tf = 0.8646;
 ctrl.tf = 0.6307
 ctrl.T = [0.0867 1.3076 -0.1228 -0.9176 -0.1555]
 ctrl.Ang = [1.5927e-16 -0.3204 -1.6626 0.0270 -0.0777]
 
 
[t, z, u, indices] = hybrid_simulation(z0,ctrl,p,[0 tf]);

% % % % setup and solve nonlinear programming problem
%      problem.objective = @(x) objective(x,z0,p);     % create anonymous function that returns objective
%      problem.nonlcon = @(x) constraints(x,z0,p);     % create anonymous function that returns nonlinear constraints
%      problem.x0 = [tf ctrl.tf ctrl.T ctrl.Ang];                   % initial guess for decision variables
%      problem.lb = [0.6 0.6 -1.4*ones(size(ctrl.T)),-0.9*pi*ones(size(ctrl.Ang))];     % lower bound on decision variables
%      problem.ub = [1.0  1.0  1.4*ones(size(ctrl.T)),0.9*pi*ones(size(ctrl.Ang))];     % upper bound on decision variables
%      problem.Aineq = []; problem.bineq = [];         % no linear inequality constraints
%      problem.Aeq = []; problem.beq = [];             % no linear equality constraints
%      problem.options = optimset('Display','iter');   % set options
%      problem.solver = 'fmincon';                     % required
%      disp('approaching station');
%      x = fmincon(problem);                           % solve nonlinear programming problem
%      
%      geebs = x;
%     
%  pause
%     
% 
% tf = geebs(1);                                        % simulation final time
% ctrl.tf = geebs(2);                                  % control time points
% ctrl.T = geebs(3:7);
% ctrl.Ang = geebs(8:12);% control values
% 
% [t, z, u, indices] = hybrid_simulation(z0,ctrl,p,[0 tf]); % run simulation
disp('Here');
% Plot COM for your submissions
% p   = [m1 m2 m3 I1 I2 I3 c1 c2 l1 l2 g]';        % parameters
figure(1)
l1 = p(9);
l2 = p(10);
c1 = p(7);
c2 = p(8);
m1 = p(1);
m2 = p(2);
m3 = p(3);
x = z(1,:);
y = z(2,:);
th1 = z(3,:);
th2 = z(4,:); % changed these
ycm = (((y+c1*sin(th1))*m1) + ((y+l1*sin(th1))*m2) + ((y+l1*sin(th1)+c2*sin(th1+th2))*m3))/(m1+m2+m3);
xcm = (((x+c1*cos(th1))*m1) + ((x+l1*cos(th1))*m2) + ((x+l1*cos(th1)+c2*cos(th1+th2))*m3))/(m1+m2+m3);

time = t;%z(5,:);
plot(time,ycm);
xlabel('Time (s)')
ylabel('Height of center of mass (m)');
title('Trajectory of center of mass');

% Run the animation
figure(2)                          % get the coordinates of the points to animate
speed = 0.25;                                 % set animation speed
clf                                         % clear fig
animate_simple(t,z,p,speed)                 % run animation