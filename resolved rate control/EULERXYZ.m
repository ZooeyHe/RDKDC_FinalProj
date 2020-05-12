function r = EULERXYZ(theta)
%Rotation matrix of Euler angles
%theta:3-vector of angles(in radians)

r1=ROTX(theta(1,1));
r2=ROTY(theta(2,1));
r3=ROTZ(theta(3,1));
r=r1*r2*r3;
end

