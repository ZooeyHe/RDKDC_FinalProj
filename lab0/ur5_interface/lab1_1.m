ur5 = ur5_interface();
goal = [ur5.home + [0 0 0 0 0 pi]', ur5.home + [0 0 0 0 pi pi]', ur5.home + [0 0 0 pi/3 pi pi]', ur5.home + [pi/2 0 pi/4 pi/3 pi pi]'];

ur5.move_joints(ur5.home,10);
pause(12);

%move 1
ur5.move_joints(goal(:,1),10);
disp("DONE with move 1");
pause(12);
%move 2
ur5.move_joints(goal(:,2),10);
disp("DONE with move 2");
pause(12);
%move 3
ur5.move_joints(goal(:,3),10);
disp("DONE with move 3");
pause(12);
%move 4
ur5.move_joints(goal(:,4),10);
disp("DONE with move 4");
pause(12);
