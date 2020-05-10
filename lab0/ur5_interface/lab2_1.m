% Zooey He | Lab02 | RDKDC

rosinit;

% The three transformations for a, b, c
g0a = [EULERXYZ([pi/2, pi/3, pi/2]), [ .1 .1  .1]'; [0 0 0 1]];
gab = [EULERXYZ([pi/3, pi/3, pi/4]), [-.2 .4  .1]'; [0 0 0 1]];
gbc = [EULERXYZ([pi/6, pi/2, pi/6]), [ .7 .6 -.1]'; [0 0 0 1]];

%Setting up the frames in RVIZ
Frame_A = tf_frame('base_link', 'Frame_A', g0a);
pause(0.5);
Frame_B = tf_frame('Frame_A', 'Frame_B', gab);
pause(0.5);
Frame_C = tf_frame('base_link', 'Frame_C', eye(4));
pause(0.5);
Frame_C.move_frame('Frame_B', gbc);

% Computing gac
gac = gab*gbc;
pause(2);

% Retreiving gacreal
gacreal = Frame_C.read_frame('Frame_A');
pause(0.5)

% Printing the two values out to compare
gacreal
gac