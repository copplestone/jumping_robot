function f = objective(x,z0,p)
% Inputs:
% x - an array of decision variables.
% z0 - the initial state
% p - simulation parameters
% 
% Outputs:
% f - scalar value of the function (to be minimized) evaluated for the
%     provided values of the decision variables.
%
% Note: fmincon() requires a handle to an objective function that accepts 
% exactly one input, the decision variables 'x', and returns exactly one 
% output, the objective function value 'f'.  It is convenient for this 
% assignment to write an objective function which also accepts z0 and p 
% (because they will be needed to evaluate the objective function).  
% However, fmincon() will only pass in x; z0 and p will have to be
% provided using an anonymous function, just as we use anonymous
% functions with ode45().
    
    %tf = x(1);
    %ctrl.tf = x(2);
    %ctrl.T = [x(3) x(4) x(5)];  
    %[t, z, u, indices] = hybrid_simulation(z0,ctrl,p,[0 tf]);
    %j = z(:,end);
    %com = COM_jumping_leg(j,p);
    
    
    % To maximise height of the jump
    %f = -com(2);
   
    % To get the robot to jump to 0.4m as fast as possible
    %f = t(end);
                                       
    % To minimise the power required to get the robot to 0.4m
    f = j(end);
end