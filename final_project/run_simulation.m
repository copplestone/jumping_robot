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
z0 = [0; pi/9; 0 ;0; 0];                    % set initial state
% Note: 5th state is the integral of torque squared over time
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

figure(1)
l = p(1);
c1 = p(2);
c2 = p(3);
m1 = p(4);
m2 = p(5);
mh = p(6);
y = z(1,:);
th = z(2,:);
ycm = (((y+c1*sin(th))*m1) + ((y+(l+c2)*sin(th))*m2) + ((y+2*l*sin(th))*mh))/(m1+m2+mh);
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