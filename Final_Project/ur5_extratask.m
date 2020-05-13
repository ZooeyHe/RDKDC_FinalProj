% Author:Boyang Xiao
% Date:2020-05-13
% Script for extra task: Control ur5 to draw a circle
clear;
cd 'RR&TJ control'/;

% Input the location of the center of the circle and the radius
x0=input("Please input the x coordinate of the center of the circle: ");
y0=input("Please input the y coordinate of the center of the circle: ");
r=input("Please input the radius of the circle: ");

% Initialization
ur5=ur5_interface();
home_config=[0 -pi/3 pi/3 -pi/2 -pi/2 0]';
ur5.move_joints(home_config,10);
pause(11);
k=1;
p0=[r+x0 y0]';
g_init=find_g(p0);

% Draw the circle
for i=1:50
    theta1=(i-1)*pi/25;
    x1=r*cos(theta1)+x0;
    y1=r*sin(theta1)+y0;
    p1=[x1 y1]';
    g1=find_g(p1);
    error=ur5RRcontrol_e(g1,k,ur5);
end

ur5.move_joints(home_config,10);
pause(11);
