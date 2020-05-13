clear;

% Move ur5 to home configuration
ur5=ur5_interface();
home_config=[pi/4 -pi/3 pi/3 -pi/2 -pi/2 0]';
ur5.move_joints(home_config,10);
pause(11);

% Input the start location and target location
x1=input("Please input the x coordinate of the start location: ");
y1=input("Please input the y coordinate of the start location: ");
x2=input("Please input the x coordinate of the target location: ");
y2=input("Please input the y coordinate of the target location: ");
p1=[x1 y1]';
p2=[x2 y2]';

% Calculate the pose of end-effector
g_start_int=find_g_int(p1);
g_start=find_g(p1);
g_target_int=find_g_int(p2);
g_target=find_g(p2);

% Select mode: 
% Inverse kinematics=1
% Resolved-rate control=2
% Transpose-Jacobian control=3
mode=input("Please select mode(Inverse kinematics=1, Resolved-rate control=2, Transpose-Jacobian control=3): ");

% Move-And-Place Task
if mode==1 % Inverse kinematics
    
elseif mode==2 %Resolved-rate control
    k=1;
    error1=ur5RRcontrol(g_start_int,k,ur5);
    pause(1);
    error2=ur5RRcontrol(g_start,k,ur5);
    pause(1);
    error3=ur5RRcontrol(g_target_int,k,ur5);
    pause(1);
    error4=ur5RRcontrol(g_target,k,ur5);
    pause(1);
elseif mode==3 % Transpose-Jacobian control
    k=2;
    error1=ur5TJcontrol(g_start_int,k,ur5);
    pause(1);
    error2=ur5TJcontrol(g_start,k,ur5);
    pause(1);
    error3=ur5TJcontrol(g_target_int,k,ur5);
    pause(1);
    error4=ur5TJcontrol(g_target,k,ur5);
    pause(1);
end
