function Jb = ur5BodyJacobian(ths)
%Adaptations used to convert HW06 Figure1 setup to RVIZ simulation setup
ths(2) = ths(2)+pi/2;
ths(4) = ths(4)+pi/2;

% Defintions
L0 = 0.09; %Height of first revolute joint from table
L1 = 0.425;
L2 = 0.392;
L3 = 0.1093;
L4 = 0.09475;
L5 = 0.0825;

%These are modified from HW6 to adapt to RVIZ setup
xi1  = [cross(-[0,0,1],[0,0,L0])'; [0,0,1]'];
xi2  = [cross(-[0,1,0],[0,0,L0])'; [0,1,0]'];
xi3  = [cross(-[0,1,0],[0,0,L0+L1])'; [0,1,0]'];
xi4  = [cross(-[0,1,0],[0,0,L0+L1+L2])'; [0,1,0]'];
xi5  = [cross(-[0,0,1],[0,L3,0])'; [0,0,1]'];
xi6  = [cross(-[0,1,0],[0,0,L0+L1+L2+L4])'; [0,1,0]'];
%gst0 is measured directly from RVIZ by using move_joints([0 0 0 0 0 0]')
%and reading the base_link to tool0 transformation.
gst0 = [[1 0 0; 0 0 1; 0 -1 0], [0 0.1914 1.011]'; [0 0 0 1]];

xi1t = adinv(exptwist(xi1,ths(1))*exptwist(xi2,ths(2))*exptwist(xi3,ths(3))*exptwist(xi4,ths(4))*exptwist(xi5,ths(5))*exptwist(xi6,ths(6))*gst0)*xi1;
xi2t = adinv(exptwist(xi2,ths(2))*exptwist(xi3,ths(3))*exptwist(xi4,ths(4))*exptwist(xi5,ths(5))*exptwist(xi6,ths(6))*gst0)*xi2;
xi3t = adinv(exptwist(xi3,ths(3))*exptwist(xi4,ths(4))*exptwist(xi5,ths(5))*exptwist(xi6,ths(6))*gst0)*xi3;
xi4t = adinv(exptwist(xi4,ths(4))*exptwist(xi5,ths(5))*exptwist(xi6,ths(6))*gst0)*xi4;
xi5t = adinv(exptwist(xi5,ths(5))*exptwist(xi6,ths(6))*gst0)*xi5;
xi6t = adinv(exptwist(xi6,ths(6))*gst0)*xi6;

Jb = [xi1t, xi2t, xi3t, xi4t, xi5t, xi6t];
end