% Zooey He | Lab02 | RDKDC
clear;

%Test (a)
q = [-pi/6, -pi/3, pi/6, -pi/2, pi/6, pi/6]';
%Calculating using our funciton
g = ur5FwdKin(q);
%Testing our function's accuracy by moving the robot.
fwdKinToolFrame = tf_frame('base_link', 'fwdKinToolFrame', eye(4));
pause(0.5);
fwdKinToolFrame.move_frame('base_link', g);
pause(1);
fwdKinToolFrame.read_frame('base_link');
ur5 = ur5_interface();
pause(0.5);
ur5.move_joints(q, 4);
pause(5);

%Test (b)

%Test (c)
ur5.move_joints(ur5.home, 3);
pause(1);
ur5.move_joints(ur5.home-[0 0 pi/4 0 0 0]', 3);
mScores = [];
for i=-pi/4:0.05:pi/4
    test_q = ur5.home;
    test_q(3) = i;
    ur5.move_joints(test_q, 2);
    pause(1);
    Jb = ur5BodyJacobian(ur5.get_current_joints());
    pause(1);
    mScores = [mScores; i, manipulability(Jb, 'sigmamin'), manipulability(Jb, 'detjac'),manipulability(Jb, 'invcond')];
end
plot(mScores(:,1), mScores(:,2:end));
%mScores contains the manipulability scores with respect to theta3