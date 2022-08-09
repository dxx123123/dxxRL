function dxdt = parkingVehicleStateFcnRRT(x, u)
% 泊车状态方程
% 状态变量x、y和偏航角θ


%%
% Parameters
wb = 2.8;

% Variables
theta = x(3);
v = u(1);
delta = u(2);

% State equations
dxdt = zeros(3,1);
dxdt(1) = v*cos(theta);
dxdt(2) = v*sin(theta);
dxdt(3) = v/wb*tan(delta);
