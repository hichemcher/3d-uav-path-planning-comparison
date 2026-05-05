function total = turning_radius(path)
% Author: H.CHERIET USTO-MB

total = 0;

for i=2:size(path,1)-1
    A = path(i-1,:);
    B = path(i,:);
    C = path(i+1,:);

    v1 = B-A;
    v2 = C-B;

    angle = acos(dot(v1,v2)/(norm(v1)*norm(v2)));
    total = total + angle;
end

end