function L = path_length(path)
% Author: H.CHERIET USTO-MB

L = 0;
for i=1:size(path,1)-1
    L = L + norm(path(i,:) - path(i+1,:));
end

end