function xip=Xip(G)

xip=zeros(6,1);
xip(1:3,1)=G(1:3,4);
xip(4,1)=G(3,2);
xip(5,1)=G(1,3);
xip(6,1)=G(2,1);

end
