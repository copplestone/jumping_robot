%disp(z(1,:));
    l = p(1);
    c1 = p(2);
    c2 = p(3);
    m1 = p(4);
    m2 = p(5);
    mh = p(6);
    I1 = p(7);
    I2 = p(8);
    g = p(9);
y = z(1,:);
th = z(2,:);


h = COM_jumping_leg(z0,p);

[t, z, u, indices] = hybrid_simulation(z0,ctrl,p,[0 tf]);
%disp("Z(end)");
%disp(size(z(:,end)));

disp(size(z));

function y = COM(z,p)
  res = COM_jumping_leg(z,p);
  y = -res(2);
end
    

