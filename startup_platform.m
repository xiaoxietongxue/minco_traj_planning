function startup_platform(opts)
%STARTUP_PLATFORM 初始化机械臂轨迹规划平台：路径、机器人模型、仿真参数、C 自定义代码配置。
%   startup_platform                       常规启动
%   startup_platform(RebuildVisual=true)   更换机器人后重新生成 Simscape 可视化子系统

arguments
    opts.RebuildVisual (1,1) logical = false
end

root = fileparts(mfilename("fullpath"));
modelsDir = fullfile(root, "models");
buildDir = fullfile(root, "build");
for d = [modelsDir, buildDir]
    if ~isfolder(d)
        mkdir(d);
    end
end
addpath(root, fullfile(root, "config"), fullfile(root, "robot"), ...
    fullfile(root, "minco"), modelsDir, fullfile(root, "scripts"), ...
    fullfile(root, "tools"), buildDir);

cfg = platform_config();
robot = robot_definition(cfg.robot, buildDir);

if opts.RebuildVisual || ~isfile(fullfile(modelsDir, "RobotVisual.slx"))
    build_robot_visual(robot, modelsDir);
end

P = make_params(cfg, robot);
assignin("base", "P", P);
assignin("base", "robotRBT", robot.rbt);
assignin("base", "PlanMode", cfg.planMode);
assignin("base", "VC_JointSpace", Simulink.Variant("PlanMode == 1"));
assignin("base", "VC_CartesianSpace", Simulink.Variant("PlanMode == 2"));
assignin("base", "platformCfg", cfg);
assignin("base", "platformRobot", robot);

mdl = "RobotTrajPlatform";
if isfile(fullfile(modelsDir, mdl + ".slx"))
    load_system(mdl);
    configure_custom_code(mdl, root);
    configure_robot_blocks(mdl, robot);
    cs = getActiveConfigSet(mdl);
    if cs.isValidParam("SimMechanicsOpenEditorOnUpdate")
        wasDirty = get_param(mdl, "Dirty");
        set_param(cs, "SimMechanicsOpenEditorOnUpdate", onOff(cfg.vis.openMechanicsExplorer));
        set_param(mdl, Dirty=wasDirty);
    end
end

modeNames = ["关节空间", "笛卡尔空间"];
fprintf("平台已初始化：机器人 %s，末端 %s，规划模式 %s，Ts = %g s\n", ...
    cfg.robot.source, robot.eeName, modeNames(cfg.planMode), cfg.Ts);
end

function configure_robot_blocks(mdl, robot)
% 把机器人相关的坐标系名同步到所有 Robotics System Toolbox 块（含未激活的变体）
wasDirty = get_param(mdl, "Dirty");
findRst = @(type) find_system(mdl, MatchFilter=@Simulink.match.allVariants, ...
    LookUnderMasks="all", ReferenceBlock="robotmaniplib/" + type);
for b = [findRst("Inverse Kinematics"); findRst("Get Jacobian")].'
    set_param(b{1}, RigidBodyTree="robotRBT", EndEffector=robot.eeName);
end
for b = findRst("Get Transform").'
    set_param(b{1}, RigidBodyTree="robotRBT", SourceBody=robot.eeName, ...
        TargetBody=robot.rbt.BaseName);
end
set_param(mdl, Dirty=wasDirty);
end

function P = make_params(cfg, robot)
maxDim = 6;
maxWpts = 16;
n = robot.numJoints;

P.Ts = cfg.Ts;
P.timeScale = cfg.timeScale;
P.endHold = cfg.endHold;
P.maxDim = maxDim;
P.maxWpts = maxWpts;
P.qMin = robot.qMin(:);
P.qMax = robot.qMax(:);
P.vMax = robot.vMax(:);
P.aMax = robot.aMax(:);

qW = deg2rad(cfg.joint.waypointsDeg).';
checkWaypoints(qW, n, maxWpts, "关节");
outOfRange = qW < P.qMin | qW > P.qMax;
if any(outOfRange, "all")
    [j, k] = find(outOfRange, 1);
    error("startup_platform:jointLimit", "关节路径点 %d 的关节 %d 超出关节限位。", k, j);
end
P.joint.wpts = padWaypoints(qW, maxDim, maxWpts);
P.joint.numWpts = int32(size(qW, 2));
P.joint.vMax = padVector(P.vMax, maxDim);
P.joint.aMax = padVector(P.aMax, maxDim);

cW = cfg.cart.waypoints.';
checkWaypoints(cW, 6, maxWpts, "笛卡尔");
cW(4:6, :) = deg2rad(cW(4:6, :));
P.cart.wpts = padWaypoints(cW, maxDim, maxWpts);
P.cart.numWpts = int32(size(cW, 2));
P.cart.vMax = cfg.cart.vMax(:);
P.cart.aMax = cfg.cart.aMax(:);
P.cart.qInit = deg2rad(cfg.cart.qInitDeg(:));
P.cart.ikWeights = cfg.cart.ikWeights(:).';
end

function checkWaypoints(W, rows, maxWpts, label)
if size(W, 1) ~= rows
    error("startup_platform:wptDim", "%s路径点每行应有 %d 个元素。", label, rows);
end
if size(W, 2) < 2 || size(W, 2) > maxWpts
    error("startup_platform:wptCount", "%s路径点数量应在 2~%d 之间。", label, maxWpts);
end
end

function M = padWaypoints(W, maxDim, maxWpts)
M = zeros(maxDim, maxWpts);
M(1:size(W, 1), 1:size(W, 2)) = W;
end

function v = padVector(x, n)
v = zeros(n, 1);
v(1:numel(x)) = x;
end

function s = onOff(tf)
s = "off";
if tf
    s = "on";
end
end
