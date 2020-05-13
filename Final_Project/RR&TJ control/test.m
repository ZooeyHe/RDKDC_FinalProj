clear;

ur5=ur5_interface();
home_config=[pi/4 -pi/3 pi/3 -pi/2 -pi/2 0]';
ur5.move_joints(home_config,10);
pause(11);

k=1.5;
x1=input("Please input the x coordinate of the start location");
y1=input("Please input the y coordinate of the start location");
x2=input("Please input the x coordinate of the target location");
y2=input("Please input the y coordinate of the target location");
p1=[x1 y1]';
p2=[x2 y2]';
g_start_int=find_g_int(p1);
g_start=find_g(p1);
g_target_int=find_g_int(p2);
g_target=find_g(p2);
error1=ur5RRcontrol(g_start_int,k,ur5);
pause(1);
error2=ur5RRcontrol(g_start,k,ur5);
pause(1);
error3=ur5RRcontrol(g_target_int,k,ur5);
pause(1);
error4=ur5RRcontrol(g_target,k,ur5);
pause(1);