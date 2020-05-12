function Jb = ur5BodyJacobian(theta)
%calculat body jacobian of ur5

[rows, cols] = size(theta);
if ((rows ~= 6) | (cols ~= 1))
  error('ur5BodyJacobian requires a 6x1 vector. Check your dimensions.');
end

theta1=theta(1,1);
theta2=theta(2,1);
theta3=theta(3,1);
theta4=theta(4,1);
theta5=theta(5,1);
theta6=theta(6,1);

l0=0.0892;
l1=0.425;
l2=0.392;
l3=0.1093;
l4=0.09475;
l5=0.0825;

xi1=[0 0 0 0 0 1]';
xi2=[-l0 0 0 0 1 0]';
xi3=[-l0-l1 0 0 0 1 0]';
xi4=[-l0-l1-l2 0 0 0 1 0]';
xi5=[l3 0 0 0 0 1]';
xi6=[-l0-l1-l2-l4 0 0 0 1 0]';

g0=[eye(3) [0 0.191 1.001]';0 0 0 1];
gst=exp_map(xi1,theta1)*exp_map(xi2,theta2)*exp_map(xi3,theta3)*exp_map(xi4,theta4)*exp_map(xi5,theta5)*exp_map(xi6,theta6)*g0;

xi2p=adjoint(exp_map(xi1,theta1))*xi2;
xi3p=adjoint(exp_map(xi1,theta1)*exp_map(xi2,theta2))*xi3;
xi4p=adjoint(exp_map(xi1,theta1)*exp_map(xi2,theta2)*exp_map(xi3,theta3))*xi4;
xi5p=adjoint(exp_map(xi1,theta1)*exp_map(xi2,theta2)*exp_map(xi3,theta3)*exp_map(xi4,theta4))*xi5;
xi6p=adjoint(exp_map(xi1,theta1)*exp_map(xi2,theta2)*exp_map(xi3,theta3)*exp_map(xi4,theta4)*exp_map(xi5,theta5))*xi6;

Js=[xi1 xi2p xi3p xi4p xi5p xi6p];
Jb=adjoint(FINV(gst))*Js;

end

