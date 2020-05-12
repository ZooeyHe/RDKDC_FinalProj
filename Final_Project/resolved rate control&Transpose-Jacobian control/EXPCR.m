function R = EXPCR(x)
%Accept 3x1 vector and return the 3x3 rotation matrix

[rows, cols] = size(x);
if ((rows ~= 3) | (cols ~= 1))
  error('EXPCR requires a 3x1 vector. Check your dimensions.');
end

theta=sqrt(x(1,1)^2+x(2,1)^2+x(3,1)^2);
xhat=SKEW3(x);
if (theta==0)
    R=eye(3);
else
    R=eye(3)+sin(theta)/theta*xhat+(1-cos(theta))/theta^2*xhat^2;
end

end

