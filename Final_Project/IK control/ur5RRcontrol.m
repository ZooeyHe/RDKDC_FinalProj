function finalerr = ur5RRcontrol(gdesired, K, ur5)
T = 1;
finalerr = 0;
k = 0;
while (finalerr == 0)
    k=k+1;
    q = ur5.get_current_joints();
    gst = ur5FwdKin(q);
    Jb = ur5BodyJacobians(q);
    xik = getXi(gdesired\gst);
    if (norm(xik(1:3))<0.05 && norm(xik(4:6))<15*pi/180)
        finalerr = norm(xik(1:3));
    elseif manipulability(Jb, 'sigmamin')<0.05 || manipulability(Jb, 'detjac')<0.05 || manipulability(Jb, 'invcond')<0.05
        finalerr = -1;
    else
        qnew = q - K*T*Jb\xik;
        ur5.move_joints(qnew, T);
    end
end
end