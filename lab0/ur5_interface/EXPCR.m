function rotmat = EXPCR(x)
if x == [0 0 0]
    rotmat = eye(3);
else
    th = sqrt(x(1)^2 + x(2)^2 + x(3)^2);
    w = x/th;
    skw = SKEW3(w);
    rotmat = eye(3)+skw*sin(th)+skw^2*(1-cos(th));
end
end