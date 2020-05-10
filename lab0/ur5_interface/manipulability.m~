function mu = manipulability(J, measure)
if strcmp('sigmamin',measure) == 1
    e = eig(J'*J);
    mu = sqrt(min(e));
elseif strcmp('detjac', measure) == 1
    mu = det(J);
elseif strcmp('invcond', measure) == 1
    e = eig(J'*J);
    mu = sqrt(min(e))/sqrt(max(e));
end
end