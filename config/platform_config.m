function cfg = platform_config()
%PLATFORM_CONFIG 平台的全部可调参数。仿真前由 startup_platform 读取。

%% 机器人
% "placeholder_ur5e"：内置 UR5e 占位；"urdf"：导入 cfg.robot.urdfFile（如 ROKAE 官方包）
cfg.robot.source = "placeholder_ur5e";
cfg.robot.urdfFile = "";
cfg.robot.eeName = "tool0";
% 规划用关节限幅；留空则使用 robot_definition 中的默认值
cfg.robot.vMax = [];
cfg.robot.aMax = [];

%% 规划模式：1 = 关节空间（MoveJ 式），2 = 笛卡尔空间（MoveL 式，位置 + 姿态）
cfg.planMode = 1;
cfg.Ts = 0.004;           % 规划/IK 采样周期 [s]
cfg.timeScale = 1.1;      % >=1，在贴合限幅的时间分配上再留的裕量
cfg.endHold = 0.5;        % 轨迹结束后继续仿真的时间 [s]

%% 关节空间路径点 [deg]，每行一个路径点（含起点和终点）
cfg.joint.waypointsDeg = [
      0  -90   90  -90  -90    0
     40  -70   80 -100  -90   20
    -35 -110  100  -80  -90  -30
     10  -60   50  -80  -90   60
      0  -90   90  -90  -90    0
    ];

%% 笛卡尔空间路径点，每行 [x y z(m)  yaw pitch roll(deg, ZYX)]，相对基座
% MINCO 曲线在急转处会向内侧弯，路径点应与基座轴线保持足够距离，避开肩部奇异
cfg.cart.waypoints = [
    0.45  0.13  0.35   -90  0  180
    0.50 -0.20  0.20   -60  0  180
    0.35 -0.35  0.45  -110  0  180
    0.50  0.00  0.40   -90  0  180
    0.40  0.30  0.30   -90  0  180
    0.45  0.13  0.35   -90  0  180
    ];
cfg.cart.vMax = [0.25 0.25 0.25 1.0 1.0 1.0];   % [m/s ×3, rad/s ×3]
cfg.cart.aMax = [0.8 0.8 0.8 3.0 3.0 3.0];      % [m/s^2 ×3, rad/s^2 ×3]
cfg.cart.qInitDeg = [0 -92 113 -111 -90 -90];   % 第一个路径点附近的 IK 初值
cfg.cart.ikWeights = [1 1 1 1 1 1];             % [姿态 ×3, 位置 ×3]

%% 可视化
cfg.vis.openMechanicsExplorer = true;
end
