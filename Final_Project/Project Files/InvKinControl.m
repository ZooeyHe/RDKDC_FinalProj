function error = InvKinControl(start, target)
% Calculating the home position
temp1 = (EULERXYZINV(start(1:3,1:3))+EULERXYZINV(target(1:3,1:3)))/2;
temp2 = (start(1:3,4)+target(1:3,4))/2;
cartesian_home = [EULERXYZ(temp1), temp2; [0 0 0 1]];

% Checking Input Validity
if ~checkCollision(start) || ~checkCollision(target) || ~checkCollision(cartesian_home)
    msg ='InvKinControl function: Invalid input - start, target, or home is below table';
    error(msg);
end

% Setup: Establishing Connection, Initializing Variables
rosinit;
ur5 = ur5_interface();
dx = 0.005;
hover_height = 0.04;
above_start = start;
above_start(3,4) = start(3,4) + hover_height;
above_target = target;
above_target = target(3,4) + hover_height;

% Step 0: Moving to home configuration
disp("Step 0: Moving to home");
joint_home = ur5InvKin(cartesian_home);
ur5.move_joints(joint_home, 5);
joint_pos = ur5.get_current_joints();

% Step 1: Moving to above the start config
init = [cartesian_home(1:3,4); EULERXYZINV(cartesian_home(1:3,1:3))];
goal = [above_start(1:3,4); EULERXYZINV(above_start(1:3,1:3))];
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
    pause(0.5);
    joint_pos = ur5.get_current_joints();
end

% Step 2: Moving to the start config
init = [above_start(1:3,4); EULERXYZINV(above_start(1:3,1:3))];
goal = [start(1:3,4); EULERXYZINV(start(1:3,1:3))];
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
    pause(0.5);
    joint_pos = ur5.get_current_joints();
end


% Step 3: Moving to above the start again
init = [start(1:3,4); EULERXYZINV(start(1:3,1:3))];
goal = [above_start(1:3,4); EULERXYZINV(above_start(1:3,1:3))];
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
    pause(0.5);
    joint_pos = ur5.get_current_joints();
end


% Step 4: Moving to above the target config
init = [above_start(1:3,4); EULERXYZINV(above_start(1:3,1:3))];
goal = [above_target(1:3,4); EULERXYZINV(above_target(1:3,1:3))];
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
    pause(0.5);
    joint_pos = ur5.get_current_joints();
end


% Step 5: Moving to the target config
init = [above_target(1:3,4); EULERXYZINV(above_target(1:3,1:3))];
goal = [target(1:3,4); EULERXYZINV(target(1:3,1:3))];
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
    pause(0.5);
    joint_pos = ur5.get_current_joints();
end

% Step 6: Moving back to above target
init = [target(1:3,4); EULERXYZINV(target(1:3,1:3))];
goal = [above_target(1:3,4); EULERXYZINV(above_target(1:3,1:3))];
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
    pause(0.5);
    joint_pos = ur5.get_current_joints();
end
end