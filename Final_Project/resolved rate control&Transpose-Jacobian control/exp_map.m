function g = exp_map(xi,theta)
%Calculat the exponential map from se(3) to SE(3)

[rows, cols] = size(xi);
if ((rows ~= 6) | (cols ~= 1))
  error('exponential_map requires a 6x1 vector. Check your dimensions.');
end

v=xi(1:3,1);
omega=xi(4:6,1);

if omega==zeros(3,1)
    g=[eye(3) v*theta;0 0 0 1];
else
    omegahat=SKEW3(omega);
    g=[expm(omegahat*theta) (eye(3)-expm(omegahat*theta))*omegahat*v;0 0 0 1];
end

end

