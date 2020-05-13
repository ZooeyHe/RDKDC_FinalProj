function xi = getXi(g)
%Take a homogenous transformation matrix and extract the unscaled twist

[rows, cols] = size(g);
if ((rows ~= 4) | (cols ~= 4))
  error('getXi requires a 4x4 matrix. Check your dimensions.');
end

r=g(1:3,1:3);
p=g(1:3,4);
theta=acos((trace(r)-1)/2);
omega=zeros(3,1);

if theta==0
    xi=[p' 0 0 0]';
else
    omega(1,1)=(r(3,2)-r(2,3))*0.5/sin(theta);
    omega(2,1)=(r(1,3)-r(3,1))*0.5/sin(theta);
    omega(3,1)=(r(2,1)-r(1,2))*0.5/sin(theta);
    v=inv((eye(3)-r)*SKEW3(omega)+omega*omega'*theta)*p;
    xi=[v;omega]*theta;
end

