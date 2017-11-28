% p   = [m1 m2 m3 I1 I2 I3 c1 c2 l1 l2 g]';        % parameters


function p = parameters() 
 l1 = .20;
 l2 = 0.2;
 m1  = .3;
 m2 = .3;
 m3 = .3;
 I1 = m1*l1^2/12;
 I2 = 0;
 I3 = m3*l2^2/12;
 c1 = l1/3;
 c2 = 2*l2/3;
 g = 9.81;
 p = [m1; m2; m3; I1; I2; I3; c1; c2; l1; l2; g];
end