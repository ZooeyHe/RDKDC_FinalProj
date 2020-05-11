function g = exptwist(xi, th)
w = xi(4:6);
v = xi(1:3);

rot = EXPCR(w*th);
trans = (eye(3)-rot)*cross(w, v);

g = [rot, trans; [0 0 0 1]];
end