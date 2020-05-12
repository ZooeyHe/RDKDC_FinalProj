function R = ROTZ(theta)
% Rotation matrix while rotating along Z axis
% theta is rotation angle

R=[cos(theta) -sin(theta) 0;sin(theta) cos(theta) 0;0 0 1];
end
