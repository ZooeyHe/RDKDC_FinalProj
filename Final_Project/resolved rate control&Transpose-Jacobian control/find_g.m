function g = find_g(a)
%According to the location, find tool frame

p1=a(1,1);
p2=a(2,1);
g=[ROTX(-pi/2) [p1 p2 0.1]';0 0 0 1];

end

