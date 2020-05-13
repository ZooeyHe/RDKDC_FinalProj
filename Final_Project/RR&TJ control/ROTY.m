function R = ROTY(theta)
% Rotation matrix while rotating along Y axis
% theta is rotation angle

R=[cos(theta) 0 sin(theta);0 1 0;-sin(theta) 0 cos(theta)];
end

