function Adg = adjoint(g)
%Adjoint operator associated with SE(3)

[rows, cols] = size(g);
if ((rows ~= 4) | (cols ~= 4))
  error('ur5FwdKin requires a 4x4 matrix. Check your dimensions.');
end

r=g(1:3,1:3);
p=g(1:3,4);
Adg=[r SKEW3(p)*r;zeros(3) r];
end

