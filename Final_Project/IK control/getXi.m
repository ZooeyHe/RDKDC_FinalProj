function xi = getXi(g)
R = g(1:3,1:3);
p = g(1:3,4);
th = acos((trace(R)-1)/2);

if th == 0
    w = [0,0,0]';
    v = p/norm(p);
    th = norm(p);
else
    w = DESKEW3(R-R')/(2*sin(th));
    v = inv(eye(3)*th + (1-cos(th))*SKEW3(w)+(th-sin(th))*SKEW3(w)^2)*p;
end
xi = [v;w]*th;
end