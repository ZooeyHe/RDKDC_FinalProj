function ahat = SKEW3(a)
%Accept a 3x1 vector and return corresponding canonical 3x3 skew-symmetric matrix 
% a:3x1 vector 
% ahat:correspongding skew-symmetric matrix

[rows, cols] = size(a);
if ((rows ~= 3) | (cols ~= 1))
  error('SKEW3 requires a 3x1 vector. Check your dimensions.');
end

ahat=zeros(3);
ahat(1,2)=-a(3,1);
ahat(1,3)=a(2,1);
ahat(2,1)=a(3,1);
ahat(2,3)=-a(1,1);
ahat(3,1)=-a(2,1);
ahat(3,2)=a(1,1);
end

