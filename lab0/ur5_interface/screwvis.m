% Zooey He | Lab02 | RDKDC

% Variables used to describe the screw motion.
xi = [1 0 1/(2*pi) 0 0 1]';

%Setting Up the frame.
Frame_D =  tf_frame('base_link', 'Frame_D', eye(4));
pause(0.5);

% Moves Frame D according to the screw xi in 120 frames of increasing theta
for i=0:2*pi/120:2*pi
   Frame_D.move_frame('base_link', exptwist(xi, i));
   pause(0.5);
end