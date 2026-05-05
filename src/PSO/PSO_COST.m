function cost = PSO_COST(sol, model)
% Cost Function with turning constraint
% Author: H.CHERIET USTO-MB

x=sol.x; y=sol.y; z=sol.z;

xs=model.start(1); ys=model.start(2); zs=model.start(3);
xf=model.end(1);   yf=model.end(2);   zf=model.end(3);

x_all = [xs x xf];
y_all = [ys y yf];
z_all = [zs z zf];

N = length(x_all);

% ---- Path length
J1 = 0;
for i=1:N-1
    J1 = J1 + norm([x_all(i+1)-x_all(i), y_all(i+1)-y_all(i), z_all(i+1)-z_all(i)]);
end

% ---- Turning constraint (<30° penalty)
J_turn = 0;
for i=2:N-1
    v1 = [x_all(i)-x_all(i-1), y_all(i)-y_all(i-1), z_all(i)-z_all(i-1)];
    v2 = [x_all(i+1)-x_all(i), y_all(i+1)-y_all(i), z_all(i+1)-z_all(i)];

    angle = acos(dot(v1,v2)/(norm(v1)*norm(v2)));

    if angle < deg2rad(30)
        J_turn = J_turn + 1000;
    end
end

% ---- Final cost
cost = 5*J1 + 10*J_turn;

end