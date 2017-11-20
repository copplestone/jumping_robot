function [cineq ceq] = constraints(x,z0,p)
% Inputs:
% x - an array of decision variables.
% z0 - the initial state
% p - simulation parameters
% 
% Outputs:
% cineq - an array of values of nonlinear inequality constraint functions.  
%         The constraints are satisfied when these values are less than zero.
% ceq   - an array of values of nonlinear equality constraint functions.
%         The constraints are satisfied when these values are equal to zero.
%
% Note: fmincon() requires a handle to an constraint function that accepts 
% exactly one input, the decision variables 'x', and returns exactly two 
% outputs, the values of the inequality constraint functions 'cineq' and
% the values of the equality constraint functions 'ceq'. It is convenient 
% in this case to write an objective function which also accepts z0 and p 
% (because they will be needed to evaluate the objective function).  
% However, fmincon() will only pass in x; z0 and p will have to be
% provided using an anonymous function, just as we use anonymous
% functions with ode45().
    tf = x(1);
    ctrl.tf = x(2);
    ctrl.T = [x(3) x(4) x(5)];
    [t, z, u, indices] = hybrid_simulation(z0,ctrl,p,[0 tf]);
    th = z(2,:);
    z_final = z(:,end);
    com = COM_jumping_leg(z_final,p);
    com_end = com(2);
    th_min = min(th);
    th_max = max(th);
    cineq = [ -th_min, th_max-pi/2];                                          
    ceq = [x(2)-x(1),com_end-0.4];                                           
                                                            
% simply comment out any alternate constraints when not in use
    
end