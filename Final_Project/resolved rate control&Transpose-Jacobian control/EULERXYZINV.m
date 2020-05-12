function a = EULERXYZINV(R)
% Accept the rotation matrix and return the euler angle
% a=[alpha beta gamma]' 
% Warning:Ifsin(beta)=1 or sin(beta)=-1, we can't get specific alpha and gamma
a=zeros(3,1);
[rows, cols] = size(R);
if ((rows ~= 3) | (cols ~= 3))
  error('EULERXYZINV requires a 3x3 matrix argument. Check your dimensions.');
end

if ((R(1,3)==1) | (R(1,3)==-1) )
    fprintf('Warning:Can not get specific euler angle.');
    error('Can not get specific euler angle.');
else
    a(2,1)=atan(R(1,3)/sqrt(R(1,1)^2+R(1,2)^2));
    a(1,1)=atan(-R(2,3)/R(3,3));
    a(3,1)=atan(-R(1,2)/R(1,1));
end
        
    
end

