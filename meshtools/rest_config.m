function spring = rest_config(spring, nodes)
% Find the rest configuration for a spring.
% Input: spring - struct with partial spring info
%        nodes - struct array with the two endpoint nodes
% Output: spring - struct with rest configurations set

dv = nodes(2).x - nodes(1).x;
spring.rij0 = [0; atan2(dv(1),dv(3)); atan2(-dv(2), sqrt(dv(1)^2 + dv(3)^2))];
spring.rji0 = spring.rij0;
spring.l0 = norm(dv);
end