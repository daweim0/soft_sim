function pe = pe_3d(x, k_n, mi, elist, qij0)
% Description: returns potential energy of mesh configuration
% Input: x - mesh state [6n+m x 1]
%        k_n - spring stiffnesses [m x 1]
%        mi - node lumped masses [n x 1]
%        elist - edge list (endpoint node indices) [m x 2]
%        qij0 - relative rest orientations of edges [3 x 2m]

% System parameters
l_0s = 0.035; % m
l_0n = 0.00914; % m
l_arm = 0.004; % m (**guess)
k_s = 142; % kg/s^2
K_th = eye(3); % kg/(rad*s^2) (**guess)
g = 9.81; % m/s^2

% Convenience
n = numel(mi);
m = size(elist, 1);
xi = reshape(x(1:3*n), 3, n);
qi = reshape(x((3*n + 1):(6*n)), 3, n);
qij = reshape(x((6*n + 1):(6*n + m)), 1, m);

PEa = zeros(1, 2*m);
PEs = zeros(1, m);
for k = 1:m
    i = elist(k, 1);
    j = elist(k, 2);
    
    a = xi(:,j) - xi(:,i);
    dx = norm(a);
    a = a/dx;
    th = acos(a(3));
    ax = cross([0;0;1], a);
    R = axang2rotm([ax' th])*eul2rotm([qij(:,k) 0 0]);
    % Node arm deflection PE
    dq = rotm2axang((eul2rotm(qi(:,i)')*eul2rotm(qij0(:,k)'))\R);
    PEa(k) = 0.5*sum(K_th*(dq(4)*dq(1:3)').^2); % small angle approximation
    dq = rotm2axang((eul2rotm(qi(:,j)')*eul2rotm(qij0(:,m+k)'))\R);
    PEa(m+k) = 0.5*sum(K_th*(dq(4)*dq(1:3)').^2); % small angle approximation
    
    % Spring stretch PE
    PEs(k) = 0.5*k_n(k)*(dx - l_0n - 2*l_arm)^2 + 0.5*k_s*(dx - l_0s - 2*l_arm)^2;
end

% Gravitational PE
PEg = g*(mi*xi(2,:)');

pe = sum(PEa) + sum(PEs) + PEg;
end