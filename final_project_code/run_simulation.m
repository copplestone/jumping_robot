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
z0 = [0; 1; 0 ;0];                    % set initial state [x,y,th1,th2]

% An equation has been added to dynamics_continuous and dynamics_discrete
% to integrate this new state.

% set guess
tf = .5;                                        % simulation final time
ctrl.tf = 0.4;                                  % control time points
ctrl.T = [1.5 1.5 1.5];                               % control values

% % setup and solve nonlinear programming problem
%  problem.objective = @(x) objective(x,z0,p);     % create anonymous function that returns objective
%  problem.nonlcon = @(x) constraints(x,z0,p);     % create anonymous function that returns nonlinear constraints
%  problem.x0 = [tf ctrl.tf ctrl.T];                   % initial guess for decision variables
%  problem.lb = [.1 .1 -2*ones(size(ctrl.T))];     % lower bound on decision variables
%  problem.ub = [1  1   2*ones(size(ctrl.T))];     % upper bound on decision variables
%  problem.Aineq = []; problem.bineq = [];         % no linear inequality constraints
%  problem.Aeq = []; problem.beq = [];             % no linear equality constraints
%  problem.options = optimset('Display','iter');   % set options
%  problem.solver = 'fmincon';                     % required
%  x = fmincon(problem);                           % solve nonlinear programming problem
%  
%  write = ['tf = ',num2str(x(1)), '  tfctrl = ',num2str(x(2)),'  T1 = ',num2str(x(3)), '  T2 = ',num2str(x(4)), '  T3 = ',num2str(x(5))];
%  disp(write);
%  % need to try and find the maximum height and the time that it occurs in
%  % order to evalutate the objective function at that height
%  height = ['Objective function at optimum = y =  ',num2str(-objective(x,z0,p)),' m'];
%  disp(height);
 
% Note that once you've solved the optimization problem, you'll need to 
% re-define tf, tfc, and ctrl here to reflect your solution.

% Highest jump
x = [0.88528 0.88528 0.11653 1.9791 1.9477];
% Fastest to 0.4m
%x = [0.21718 0.21718 2.0 2.0 2.0];
% Proper torque optimisation
%x = [0.33103 0.33103 1.3585 1.5866 -1.0734];
% Alternative torque optimisation
% x = [0.40571 0.40571 0.98753 1.5135 0.17814];
 
%  height = ['Objective function at optimum = y =  ',num2str(-objective(x,z0,p)),' m'];
%  disp(height);

tf = x(1);                                        % simulation final time
ctrl.tf = x(2);                                  % control time points
ctrl.T = [x(3) x(4) x(5)];                               % control values

[t, z, u, indices] = hybrid_simulation(z0,ctrl,p,[0 tf]); % run simulation
disp("Here");
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

time = z(5,:);
plot(time,ycm);
xlabel('Time (s)')
ylabel('Height of center of mass (m)');
title('Trajectory of center of mass');

% Run the animation
figure(2)                          % get the coordinates of the points to animate
speed = .25;                                 % set animation speed
clf                                         % clear fig
animate_simple(t,z,p,speed)                 % run animation