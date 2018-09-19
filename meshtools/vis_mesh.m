function vis_mesh(nodes, springs, x)
% Visualize a mesh.
% Input: nodes - struct array of node info
%        springs - struct array of spring info
%        x - mesh state (optional)

if (nargin < 3)
    x = [cat(1, nodes.x); zeros(3*numel(springs), 1)];
end

n = numel(nodes);

% Node positions
scatter3(x(1+3*(0:n-1)), x(2+3*(0:n-1)), x(3+3*(0:n-1)), 'bo')

for k = 1:numel(springs)
    i = springs(k).nodes(1);
    j = springs(k).nodes(2);
    
    % Spring orientations
    line(x(1+3*([i,j]-1)), x(2+3*([i,j]-1)), x(3+3*([i,j]-1)), 'Color', 'r')
    
    % Spring rest orientations relative to nodes
    ri = x(3*n + 3*(i-1) + (1:3));
    rj = x(3*n + 3*(j-1) + (1:3));
    vik = eul2rotm(ri')*eul2rotm(springs(k).rij0')*[0 0 1]';
    vjk = eul2rotm(rj')*eul2rotm(springs(k).rji0')*[0 0 -1]';
    
    len = 0.3*norm(x(3*(i-1) + (1:3)) - x(3*(j-1) + (1:3)));
    line(x(1+3*(i-1)) + [0 len*vik(1)], ...
         x(2+3*(i-1)) + [0 len*vik(2)], ...
         x(3+3*(i-1)) + [0 len*vik(3)], 'Color', 'g', 'LineWidth', 2)
    line(x(1+3*(j-1)) + [0 len*vjk(1)], ...
         x(2+3*(j-1)) + [0 len*vjk(2)], ...
         x(3+3*(j-1)) + [0 len*vjk(3)], 'Color', 'g', 'LineWidth', 2)
end
axis equal

end