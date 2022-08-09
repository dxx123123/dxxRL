
% 雷达参数
maxLidarDist = 6;           % 雷达可测量的最大距离 (m)
numSensors   = 12;          % 雷达传感器数量
obsMat = map.ObstacleMatrix;
lidarTol    = 0.5;          % 雷达测量的最小距离
% 目标姿势的误差容限
xyerrTol    = 0.75;         % 位置误差公差(m)
terrTol     = deg2rad(10);  % 方向误差公差(m)

% 相机参数
cameraDepth = 10;           % 相机深度 (m)
cameraViewAngle = deg2rad(120); % 摄像机视野 (rad)

% 采样时间，模拟时间
Ts = 0.1;                   % MPC和RL控制器的采样时间 (sec)
% Tv = 0.1;                 % sec
Tf = 50;                    % 模拟时间 (sec)

% 训练参数
speedMax = 2;               % 最大速度 (m/s)
steerMax = pi/4;            %最大转向角 (rad)
trainXBounds  = map.TrainingZoneXLimits;
trainYBounds  = map.TrainingZoneYLimits;
trainTBounds  = [-2*pi 2*pi];

% 模拟参数
xBounds  = map.XLimits;
yBounds  = map.YLimits;
tBounds  = [-Inf Inf];