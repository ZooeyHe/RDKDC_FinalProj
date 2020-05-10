function rot = EULERXYZ(vec)
    rot = ROTX(vec(1)) * ROTY(vec(2)) * ROTZ(vec(3));
end