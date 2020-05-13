function [joint_pos, error] = invKinMove(go, stop, ur5, joint_pos, dx) 
init = [go(1:3,4); EULERXYZINV(go(1:3,1:3))];
goal = [stop(1:3,4); EULERXYZINV(stop(1:3,1:3))];
numSteps = ceil(norm(goal(1:3)-init(1:3))/dx);
for i = 1:numSteps
    %Getting the ee's coordinate at time i assuming linear motion of ee.
    pos_i = init + (goal-init)*min(i/numSteps, 1);
    g_i = homoRep(pos_i(1:3), pos_i(4:6));
    joints_i = ur5InvKin(g_i);
    
    %Selecting the best inv kin result
    [~,index] = min(vecnorm(joints_i-joint_pos));
    joints_i = joints_i(:, index);
    
    %Moving joints
    ur5.move_joints(joints_i, 0.5);
    % pause(0.25);
    joint_pos = ur5.get_current_joints();
end
g = ur5FwdKin(joint_pos); % The actual ee position given by joint_pos
dso3 = sqrt(trace((g(1:3,1:3)-stop(1:3,1:3)*(g(1:3,1:3)-stop(1:3,1:3))')));
dR3 = norm(g(1:3,4)-stop(1:3,4));
error = [dR3, dso3]; % The errors
end