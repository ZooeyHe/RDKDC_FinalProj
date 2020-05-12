function error = InvKinControl(start, target)
% Calculating the home position
temp1 = (EULERXYZINV(start(1:3,1:3))+EULERXYZINV(target(1:3,1:3)))/2;
temp2 = (start(1:3,4)+target(1:3,4))/2 + [0 0 0.4]';
cartesian_home = [EULERXYZ(temp1), temp2; [0 0 0 1]];
%cartesian_home = ur5FwdKin([0 -pi/2 0 -pi/2 0 0]);


% Checking Input Validity
if ~checkCollision(start) || ~checkCollision(target) || ~checkCollision(cartesian_home)
    msg ='InvKinControl function: Invalid input - start, target, or home is below table';
    error(msg);
end

% Setup: Establishing Connection, Initializing Variables
rosshutdown;
ur5 = ur5_interface();
dx = 0.015;
hover_height = 0.2;
above_start = start;
above_start(3,4) = start(3,4) + hover_height;
above_target = target;
above_target(3,4) = target(3,4) + hover_height;

% Step 0: Moving to home configuration
disp("Step 0: Moving to home");
joint_home = ur5InvKin(cartesian_home);
ur5.move_joints(joint_home, 10);
pause(5);
joint_pos = ur5.get_current_joints();
pause(2);

% Step 1: Moving to above the start config
disp("Step 1: Moving to position above start");
joint_pos = invKinMove(cartesian_home, above_start, ur5, joint_pos, dx);

% Step 2: Moving to the start config
pause(3);
disp("Step 2: Moving to start");
joint_pos = invKinMove(above_start, start, ur5, joint_pos, dx);

% Step 3: Moving to above the start again
pause(3);
disp("Step 3: Moving back above start");
joint_pos = invKinMove(start, above_start, ur5, joint_pos, dx);

% Step 4: Moving to above the target config
pause(3);
disp("Step 4: Moving above the target");
joint_pos = invKinMove(above_start, cartesian_home, ur5, joint_pos, dx);
joint_pos = invKinMove(cartesian_home, above_target, ur5, joint_pos, dx);

% Step 5: Moving to the target config
pause(3);
disp("Step 5: Moving to target");
joint_pos = invKinMove(above_target, target, ur5, joint_pos, dx);

% Step 6: Moving back to above target
pause(3);
disp("Step 6: Moving back above target");
joint_pos = invKinMove(target, above_target, ur5, joint_pos, dx);
end