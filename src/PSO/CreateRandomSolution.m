function sol = CreateRandomSolution(VarSize,VarMin,VarMax)
% Author: H.CHERIET USTO-MB

sol.x = unifrnd(VarMin.x,VarMax.x,VarSize);
sol.y = unifrnd(VarMin.y,VarMax.y,VarSize);
sol.z = unifrnd(VarMin.z,VarMax.z,VarSize);

end