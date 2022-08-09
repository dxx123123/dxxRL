function [A, B] = parkingVehicleStateJacobianFcnRRT(x, u)
% 泊车状态方程
% 状态变量x、y和偏航角θ

%%
% Parameters
wb = 2.8;

% Variables
theta = x(3);
v = u(1);
delta = u(2);

% 在当前条件下线性化状态方程
A = zeros(3,3);
B = zeros(3,2);

A(1,3) = -v*sin(theta);
B(1,1) = cos(theta);

A(2,3) = v*cos(theta);
B(2,1) = sin(theta);

B(3,1) = 1/wb*tan(delta);
B(3,2) = 1/wb*(v*(1+tan(delta)^2));
