function dxdt = vehicleStateFcn(x, u)
% 状态方程
% 状态变量x、y和偏航角θ。
% 控制变量v和转向角.

%参数
wb = 2.8;

%变量
theta = x(3);
v = u(1);
delta = u(2);

% 状态方程
dxdt = zeros(3,1);
dxdt(1) = v*cos(theta);
dxdt(2) = v*sin(theta);
dxdt(3) = v/wb*tan(delta);
