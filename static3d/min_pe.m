% Description: find static equilibrium configuration of mesh
clear

[nodes, springs] = parse_mesh('house.xml');

% Parameters
k_min = 30.2; % kg/s^2
k_max = 718.9; % kg/s^2
m_edge = 0.1; % kg (**guess)
m_node = 0.01; % kg (**guess)

k_n = repmat(k_min, 1, 5);

% For each edge the indices two nodes it connects [m x 2]
elist = cat(1, springs.nodes);

% Split the mass of each spring between nodes [n x 1]
nEdges = arrayfun(@(x) sum(any(elist==x, 2)), 1:numel(nodes));
mi = m_node + 0.5*m_edge*nEdges;

% The rest orientation of each edge relative to its endpoints [4 x 2m]
qij0 = [springs.qij0 springs.qji0];

n = numel(mi);
m = size(elist, 1);

% Potential energy for given mesh
f_pe = @(x) pe_3d(x, k_n, mi, elist, qij0);

% Starting guess for configuration
%   node positions [3n], node orientations [3n], spring twists [m]
x_start = [cat(1, nodes.x); cat(1, nodes.q); zeros(m, 1)];

% Static nodes
lb = -inf(size(x_start));
ub = inf(size(x_start));
for i = find([nodes.static])
    lb(3*(i-1) + (1:3)) = nodes(i).x;
    ub(3*(i-1) + (1:3)) = nodes(i).x;
    lb(3*n + 3*(i-1) + (1:3)) = nodes(i).q;
    ub(3*n + 3*(i-1) + (1:3)) = nodes(i).q;
end

opt = optimoptions(@fmincon, ...
    'Algorithm', 'sqp', ...
    'MaxIterations', 1000, ...
    'MaxFunctionEvaluations', 10000, ...
    'SpecifyConstraintGradient', false, ...
    'SpecifyObjectiveGradient', false);
[x, PE, flag] = fmincon(f_pe, x_start, [], [], [], [], lb, ub, [], opt);
vis_mesh(nodes, springs, x)