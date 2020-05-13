function invmat = FINV(g)
    R = g(1:3, 1:3);
    t = g(1:3, 4);
    invmat = [R' -R'*t; [0 0 0 1]];
end