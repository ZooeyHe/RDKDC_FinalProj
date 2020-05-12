function R = ROTX(theta)
% Rotation matrix while rotating along X axis
% theta is rotation angle

R=[1 0 0;0 cos(theta) -sin(theta);0 sin(theta) cos(theta)];
end

