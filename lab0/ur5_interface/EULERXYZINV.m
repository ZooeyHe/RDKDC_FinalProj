function w = EULERXYZINV(R)
%{
Given that w = [th1 th2 th3]', th2 < 0 or th2 > pi. This is known as a
Gimble lock, and causes singularities where multiple 'w's can represent the
same rotation matrix.
%}

th2 = atan2(R(1,3), sqrt(R(1,1)^2 + R(1,2)^2));
if th2 < 0
    disp("WARNING: Calculated thetaY exceeds range, it was adjusted to [0, pi] to prevent singularities");
    th2 = th2 + pi;
    th3 = -atan2(-R(1,2)/cos(th2), R(1,1)/cos(th2));
    th1 = -atan2(-R(2,3)/cos(th2), R(3,3)/cos(th2));
else
    th3 = atan2(-R(1,2)/cos(th2), R(1,1)/cos(th2));
    th1 = atan2(-R(2,3)/cos(th2), R(3,3)/cos(th2));
end
w = [th1, th2, th3]';
end