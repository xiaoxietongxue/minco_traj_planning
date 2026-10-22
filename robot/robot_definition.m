function robot = robot_definition(cfgRobot, buildDir)
%ROBOT_DEFINITION 机器人统一定义接口。平台其余部分只通过返回的结构体访问机器人，
%   更换机器人（例如 ROKAE 官方 URDF）时只需修改 platform_config 中的 cfg.robot。
%
%   robot.rbt        rigidBodyTree（DataFormat="row"），供 IK/FK/雅可比模块使用
%   robot.urdfFile   供 Simscape 可视化导入的 URDF 绝对路径
%   robot.eeName     末端坐标系名
%   robot.jointNames 运动关节名（顺序即关节向量顺序）
%   robot.qMin/qMax/vMax/aMax  1xN 关节限幅 [rad, rad/s, rad/s^2]

arguments
    cfgRobot (1,1) struct
    buildDir (1,1) string
end

switch cfgRobot.source
    case "placeholder_ur5e"
        rbt = loadrobot("universalUR5e", DataFormat="row");
        urdfFile = exportPlaceholderUrdf(rbt, fullfile(buildDir, "placeholder_urdf"));
        vMaxDefault = pi;          % UR5e 各轴 180 deg/s
        aMaxDefault = 4.0;
    case "urdf"
        urdfFile = string(cfgRobot.urdfFile);
        if ~isfile(urdfFile)
            error("robot_definition:urdfNotFound", "URDF 文件不存在: %s", urdfFile);
        end
        urdfFile = string(java.io.File(urdfFile).getCanonicalPath());
        rbt = importrobot(urdfFile, DataFormat="row");
        vMaxDefault = [];
        aMaxDefault = 4.0;
    otherwise
        error("robot_definition:unknownSource", "未知的机器人来源: %s", cfgRobot.source);
end

if ~any(strcmp(rbt.BodyNames, cfgRobot.eeName))
    error("robot_definition:noEndEffector", "模型中不存在末端坐标系 '%s'。可选: %s", ...
        cfgRobot.eeName, strjoin(rbt.BodyNames, ", "));
end

jointNames = strings(1, 0);
qMin = [];
qMax = [];
for i = 1:rbt.NumBodies
    jnt = rbt.Bodies{i}.Joint;
    if ~strcmp(jnt.Type, "fixed")
        jointNames(end+1) = string(jnt.Name); %#ok<AGROW>
        qMin(end+1) = jnt.PositionLimits(1); %#ok<AGROW>
        qMax(end+1) = jnt.PositionLimits(2); %#ok<AGROW>
    end
end
numJoints = numel(jointNames);
if numJoints ~= 6
    error("robot_definition:dof", "平台目前按 6 轴设计，当前模型有 %d 个运动关节。", numJoints);
end

if isempty(vMaxDefault)
    vMaxDefault = urdfVelocityLimits(urdfFile, jointNames);
end

robot.rbt = rbt;
robot.urdfFile = urdfFile;
robot.eeName = string(cfgRobot.eeName);
robot.jointNames = jointNames;
robot.numJoints = numJoints;
robot.qMin = qMin;
robot.qMax = qMax;
robot.vMax = pickLimit(cfgRobot.vMax, vMaxDefault, numJoints);
robot.aMax = pickLimit(cfgRobot.aMax, aMaxDefault, numJoints);
end

function urdfFile = exportPlaceholderUrdf(rbt, outDir)
urdfFile = fullfile(outDir, "robotDataPackage", "urdf", "placeholder_robot.urdf");
if isfile(urdfFile)
    return
end
if ~isfolder(outDir)
    mkdir(outDir);
end
% exportrobot 在 ExportMesh=true 时只接受不含路径的文件名，输出到当前目录
prevDir = cd(outDir);
cleanup = onCleanup(@() cd(prevDir));
exportrobot(rbt, OutputfileName="placeholder_robot.urdf", ExportMesh=true);
end

function v = urdfVelocityLimits(urdfFile, jointNames)
txt = fileread(urdfFile);
v = pi * ones(1, numel(jointNames));
for k = 1:numel(jointNames)
    blk = regexp(txt, "<joint[^>]*name=""" + jointNames(k) + """.*?</joint>", "match", "once");
    tok = regexp(blk, "<limit[^>]*velocity=""([^""]+)""", "tokens", "once");
    if ~isempty(tok)
        v(k) = str2double(tok{1});
    end
end
end

function lim = pickLimit(userValue, defaultValue, n)
if isempty(userValue)
    lim = defaultValue;
else
    lim = userValue;
end
lim = reshape(lim, 1, []);
if isscalar(lim)
    lim = repmat(lim, 1, n);
end
end
