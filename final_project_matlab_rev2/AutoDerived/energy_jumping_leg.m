function E = energy_jumping_leg(in1,in2)
%ENERGY_JUMPING_LEG
%    E = ENERGY_JUMPING_LEG(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.0.
%    28-Nov-2017 19:12:17

I1 = in2(4,:);
I2 = in2(5,:);
I3 = in2(6,:);
Y = in1(2,:);
c1 = in2(7,:);
c2 = in2(8,:);
dth1 = in1(7,:);
dth2 = in1(8,:);
dx = in1(5,:);
dy = in1(6,:);
g = in2(11,:);
l1 = in2(9,:);
m1 = in2(1,:);
m2 = in2(2,:);
m3 = in2(3,:);
th1 = in1(3,:);
th2 = in1(4,:);
t2 = th1+th2;
t3 = cos(t2);
t9 = cos(th1);
t4 = dy+dth1.*(c2.*t3+l1.*t9)+c2.*dth2.*t3;
t5 = sin(t2);
t11 = sin(th1);
t15 = c2.*t5;
t16 = l1.*t11;
t6 = -dx+dth1.*(t15+t16)+c2.*dth2.*t5;
t7 = dth1+dth2;
t8 = dth1.^2;
t10 = dy+c1.*dth1.*t9;
t12 = dx-c1.*dth1.*t11;
t13 = dy+dth1.*l1.*t9;
t14 = dx-dth1.*l1.*t11;
E = I1.*t8.*(1.0./2.0)+I2.*t8.*(1.0./2.0)+I3.*t7.^2.*(1.0./2.0)+m3.*(t4.^2+t6.^2).*(1.0./2.0)+m1.*(t10.^2+t12.^2).*(1.0./2.0)+m2.*(t13.^2+t14.^2).*(1.0./2.0)+g.*m1.*(Y+c1.*t11)+g.*m2.*(Y+t16)+g.*m3.*(Y+t15+t16);
