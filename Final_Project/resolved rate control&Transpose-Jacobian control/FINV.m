function ginv = FINV(g)
%Accept a 4x4 homogeneous transformation and returns its matrix inverse

[rows, cols] = size(g);
if ((rows ~= 4) | (cols ~= 4))
  error('FINV requires a 4x4 vector. Check your dimensions.');
end

r=g(1:3,1:3);
p=g(1:3,4);
ginv=[r' -r'*p;[0 0 0] 1];

end

