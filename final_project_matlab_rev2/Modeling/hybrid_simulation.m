
function [tout, zout, uout, indices, sols] = hybrid_simulation(z0,ctrl,p,tspan)


%Inputs:
% z0 - the initial state [x,y,th1,th2,dx,dy,dth1,dth2]
% ctrl- control structure
% p - simulation parameters
% tspan - [t0 tf] where t0 is start time, tf is end time
% 
% Outputs:
% tout - vector of all time points
% zout - vector of all state trajectories
% uout - vector of all control trajectories
% indicies - vector of indices indicating when each phases ended
%   for instance, tout( indices(1) ) provides the time at which
%   the first phase (stance) ended
% sols - vector of solution structures
%

    t0 = tspan(1); tend = tspan(end);   % set initial and final times
    %disp(tspan)
    
    %% Setup tolerance Options
    inttol = 1e-9;  % set integration tolerances
    iphase = 3;     % phase number is currently phase 3 % begin in drop
    sols = [];      % initialize array of solution structures
    flight_counter = 0;
    
    while(flight_counter < 2)
        % now include the number of the phase in the call to he event
        % function (1== stance, 2 == flight, 3 == initial drop)
        opts = odeset('Events', @(t,z) event_conditions(t,z,ctrl,p,iphase), 'abstol',inttol,'reltol',inttol);
        f = @(t,z) dynamics_continuous(t, z, ctrl,p, iphase);    % we also include the number of the phase in the call to the EoM function
        sol = ode45(f, [t0 tend], z0, opts);    % integate until an event happens or time runs out
        sol.iphase = iphase;                    % store the phase number in the solution structure
        t0 = sol.x(end);                        % reset the integration initial time

        if isfield(sol,'ie') && ~isempty(sol.ie)
            z0 = sol.ye(:,end);                         % run the discrete dynamics function
            if any(sol.ie == 1)                         % if takeoff occured during integration
                iphase = 2;                             % phase is now phase 2
                flight_counter = flight_counter+1;
            elseif any(sol.ie == 2)                     % if touchdown occured during integration
                iphase = 1;                             % phase is now phase 1
            end
        else
            sol.ie = []; sol.xe = []; sol.ye = [];      % leave this just in case no event occured
        end
        sols = [sols; sol];                             % append solution structure to array       
    end

    % assemble the results from each solution structure in the array
    tout = []; zout = []; uout = []; indices = [];
    for ii = 1:length(sols)
        sol = sols(ii);                                     % get the next solution structure
        iphase = sol.iphase;                                % get the phase number
        sol.x(1) = sol.x(1)+1e-6;                           % the time starts just a little after the last time ended (required for animation)
        tout = [tout sol.x];                                % append time points
        zout = [zout sol.y];                                % append states
        uout = [uout control_laws(sol.x,sol.y,ctrl,p,iphase)];   % append controls
        indices = [indices length(tout)];                   % append indices at which phases end
    end

end

%% Continuous dynamics
function [dz, Fc] = dynamics_continuous(t,z,ctrl,p,iphase)

    u = control_laws(t,z,ctrl,p,iphase);  % get controls at this instant
   
    % Contact model
    if iphase == 2  % in flight phase
       Fc = zeros(6,1);
    else            % in stance phase
       %keypoints = keypoints_jumping_leg(z,p);
       pA = pointA_jumping_leg(z,p);
       pB = pointB_jumping_leg(z,p);
       pC = pointC_jumping_leg(z,p);
       
       pdA = d_pointA_jumping_leg(z,p);
       pdB = d_pointB_jumping_leg(z,p);
       pdC = d_pointC_jumping_leg(z,p);
       
       %[minPoint indexPoint] = min(keypoints(2,:));
       %C =  minPoint;%C_jumping_leg(z,p);
       %disp("PB");
       %disp(pA(2))
       
       Fc = zeros(6,1);
       if ((pA(2)) < 0) 
        Fc(1,1) = -1.0*pdA(1);
        %Fc(2,1) = max((-1500 * pA(2)- 10*pdA(2)),0);
        Fc(2,1) = max((-3500 * pA(2)- 10*pdA(2)),0);
       end
       if ((pB(2)) < 0) 
        Fc(3,1) = -1.0*pdB(1);
        %Fc(4,1) = max((-1500 * pB(2)- 10*pdB(2)),0);
        Fc(4,1) = max((-3500 * pB(2)- 10*pdB(2)),0);
       end
        if ((pC(2)) < 0) 
        Fc(5,1) = -1.0*pdC(1);
        %Fc(6,1) = max((-1500 * pC(2)- 10*pdC(2)),0);
        Fc(6,1) = max((-3500 * pC(2)- 10*pdC(2)),0);
        end
       
    end
    
    A = A_jumping_leg(z,p);                 % get full A matrix
    b = b_jumping_leg(z,u,Fc,p);               % get full b vector
    
    x = A\b;                % solve system for accelerations (and possibly forces)
    dz(1:4,1) = z(5:8); % assign velocities to time derivative of state vector
    dz(5:8,1) = x(1:4);   % assign accelerations to time derivative of state vector

    %dz(9) = 1;              % change to integrate torque squared
end

%% Control
function u = control_laws(t,z,ctrl,p,iphase)

    if iphase == 1
        % Linearily interpolate torque in stance
        ctrl.t = linspace(0,ctrl.tf,length(ctrl.T));
        u = interp1(ctrl.t,ctrl.T,t,'linear','extrap');
        
        th = z(4,:);%z(3,:)+z(4,:);            % leg angle
        dth = z(8,:);%z(7,:)+z(8,:);  
    else
        if iphase == 2
%         
%         % PD Control in flight
         th = z(4,:);%z(3,:)+z(4,:);            % leg angle
         dth = z(8,:);%z(7,:)+z(8,:);           % leg angular velocity
  
         th1 = z(3,:);
         
         if ((th1 < -pi/2) | (th1 > 0.4*pi/2))
            thd = 0;
         else
         thd = -pi/1.2;             % desired leg angle
         end
         %  thd = 0;
        k = 5;                  % stiffness (N/rad)
         b = .5;                 % damping (N/(rad/s))
% 
        ctrl.t = linspace(0,ctrl.tf,length(ctrl.Ang));
        thd = interp1(ctrl.t,ctrl.Ang,t,'linear','extrap');

         u = -k*(th-thd) - b*dth;% apply PD control
        else
          u = 0;
            % stiffness (N/rad)
         
%         %disp(th)
%        if ((th>(0.9*pi)))
%             %disp("NO")
%             u = -500*th-(0.9*pi)-3*dth;
%         end
%         
%         if ((th<(-0.9*pi)))
%             %disp("HEY")
%             u = -500*(th-(-0.9*pi))-3*dth;
%         end
%         
%         %u = 0.0;
        end
     end
    
     %ctrl.t = linspace(0,ctrl.tf,length(ctrl.T));
     %u = interp1(ctrl.t,ctrl.T,t,'linear','extrap');
       th = z(4,:);%z(3,:)+z(4,:);            % leg angle
         dth = z(8,:);%z(7,:)+z(8,:);           % leg angular velocity
     
        %teebs = linspace(0,ctrl.tf,length(ctrl.T));
        %u = interp1(teebs,ctrl.T,t,'linear','extrap');
         
     if ((th>(0.9*pi)))
            %disp("NO")
           u = -500*th-(0.9*pi)-3*dth;
     end
        
     if ((th<(-0.9*pi)))
            %disp("HEY")
          u = -500*(th-(-0.9*pi))-3*dth;
     end

end

%% Events
function [value,isterminal,direction] = event_conditions(t,z,ctrl,p,iphase)
 
       pA = pointA_jumping_leg(z,p);
       pB = pointB_jumping_leg(z,p);
       pC = pointC_jumping_leg(z,p);
       
       pdA = d_pointA_jumping_leg(z,p);
       pdB = d_pointB_jumping_leg(z,p);
       pdC = d_pointC_jumping_leg(z,p);

    if iphase == 1                      % in stance phase ( (pA(2) >0) || (pB(2) >0) || (pC > 0))
        %disp("stance")
        [x, Fc] = dynamics_continuous(t,z,ctrl,p,iphase);
        value(1) = norm(Fc);              
        isterminal(1) = 1;              % terminate integration when ground reaction force is zero
        direction(1) = -1;              % if it's decreasing
    else
        if iphase == 2
        %disp("non stance") flight after punch
        value(2) =  min([pA(2),pB(2),pC(2)]);%       % value() is the foot height
        isterminal(2) = 1;              % terminate integration when foot height is zero
        direction(2) = -1;              % if it's decreasing
        else
            value(2) =  min([pA(2),pB(2),pC(2)]);%       % value() is the foot height
            isterminal(2) = 1;              % terminate integration when foot height is zero
            direction(2) = -1;              % if it's decreasing
        end
    end
end
