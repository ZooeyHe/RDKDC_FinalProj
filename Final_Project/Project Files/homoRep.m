function g = homoRep(p, th)
% Generates the homogeneous representation from the translation and euler angles
g = [EULERXYZ(th), p;[0 0 0 1]];
end