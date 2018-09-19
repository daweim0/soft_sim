function [nodes, springs, groups] = gen_trunk(nx, ny, nz)
% Generate a trunk mesh with a cubic(ish) lattice.
% Input: lattice dimensions
% Output: struct arrays of nodes and edges in mesh and control groups

n = nx*ny*nz;
m = (nx-1)*ny*nz + nx*(ny-1)*nz + nx*ny*(nz-1) + 2*(nx-1)*(ny-1)*(nz-1);
nodes = struct('static', false, 'x', cell(n,1), 'k', 0.15, 'm', 0.002);
springs = struct('nodes', cell(m,1), 'rij0', [], 'rji0', [], 'l0', [], 'k', 6000, 'm', 0.01);
groups = struct('springs', cell(4,1));

for i = 1:4
    groups(i).springs = i + 4*(0:(nx-2))';
end

for i = 1:n
    ix = mod(i-1, nx);
    iy = mod(floor((i-1)/nx), ny);
    iz = mod(floor((i-1)/(nx*ny)), nz);
    if (ix == 0) % anchor -x end of trunk
        nodes(i).static = true;
    end
    nodes(i).x = [0.035*ix 0.02*iy 0.04*iz]';
end

idx = 1;
for ix = 0:(nx - 2)
    for iy = 0:(ny - 1)
        for iz = 0:(nz - 1)
            springs(idx).nodes = 1 + ix + nx*(iy + ny*iz) + [0 1];
            springs(idx) = rest_config(springs(idx), nodes(springs(idx).nodes));
            idx = idx + 1;
        end
    end
end

for ix = 0:(nx - 1)
    for iy = 0:(ny - 2)
        for iz = 0:(nz - 1)
            springs(idx).nodes = 1 + ix + nx*(iy + ny*iz) + [0 nx];
            springs(idx) = rest_config(springs(idx), nodes(springs(idx).nodes));
            idx = idx + 1;
        end
    end
end

for ix = 0:(nx - 1)
    for iy = 0:(ny - 1)
        for iz = 0:(nz - 2)
            springs(idx).nodes = 1 + ix + nx*(iy + ny*iz) + [0 nx*ny];
            springs(idx) = rest_config(springs(idx), nodes(springs(idx).nodes));
            idx = idx + 1;
        end
    end
end

% Diagonals beams
for ix = 0:(nx - 2)
    for iy = 0:(ny - 2)
        for iz = 0:(nz - 2)
            springs(idx).nodes = 1 + ix + nx*(iy + [0 0] + ny*(iz + [1 0])) + [0 1];
            springs(idx) = rest_config(springs(idx), nodes(springs(idx).nodes));
            idx = idx + 1;
            springs(idx).nodes = 1 + ix + nx*(iy + [1 1] + ny*(iz + [1 0])) + [0 1];
            springs(idx) = rest_config(springs(idx), nodes(springs(idx).nodes));
            idx = idx + 1;
        end
    end
end

end