function mat = adinv(g)
R = g(1:3,1:3);
t = g(1:3,4);
mat = [R', -R'*SKEW3(t); zeros(3), R'];
end