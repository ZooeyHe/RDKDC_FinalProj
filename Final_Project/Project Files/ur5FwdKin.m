function gst = ur5FwdKin(ths)

% Defintions
L0 = 0.09; %Height of first revolute joint from table
L1 = 0.425;
L2 = 0.392;
L3 = 0.1093;
L4 = 0.09475;
L5 = 0.0825;

% Used to calculate the twists
xi1  = [cross(-[0,0,1],[0,0,0])'; [0,0,1]'];
xi2  = [cross(-[1,0,0],[0,0,0])'; [1,0,0]'];
xi3  = [cross(-[1,0,0],[0,0,L1])'; [1,0,0]'];
xi4  = [cross(-[1,0,0],[0,0,L1+L2])'; [1,0,0]'];
xi5  = [cross(-[0,0,1],[L3,0,0])'; [0,0,1]'];
xi6  = [cross(-[1,0,0],[0,0,L1+L2+L4])'; [1,0,0]'];
%gst0 has the robot in a vertical configuration
gst0 = [eye(3), [L3+L5 0 L1+L2+L4]'; [0 0 0 1]];
gst = exptwist(xi1,ths(1))*exptwist(xi2,ths(2))*exptwist(xi3,ths(3))*exptwist(xi4,ths(4))*exptwist(xi5,ths(5))*exptwist(xi6,ths(6))*gst0;
end