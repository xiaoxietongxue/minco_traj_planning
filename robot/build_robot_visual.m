function build_robot_visual(robot, modelsDir)
%BUILD_ROBOT_VISUAL 由 URDF 生成 Simscape Multibody 可视化子系统 RobotVisual.slx（Subsystem Reference）。
%   输入端口 q (6x1, rad)，各关节为"输入运动 + 计算力矩"，Mechanics Explorer 中显示动画。
%   更换机器人后重新运行 startup_platform(RebuildVisual=true) 即可重新生成。

arguments
    robot (1,1) struct
    modelsDir (1,1) string
end

visualName = "RobotVisual";
importName = "RobotVisual_import";
visualFile = fullfile(modelsDir, visualName + ".slx");

closeIfOpen(visualName);
closeIfOpen(importName);
if isfile(visualFile)
    delete(visualFile);
end

smimport(char(robot.urdfFile), ModelName=char(importName));
fixMeshPaths(importName, robot.urdfFile);
mechBlocks = find_system(importName, SearchDepth=1, Type="block");

inport = add_block("simulink/Sources/In1", importName + "/q", MakeNameUnique="on");
set_param(inport, PortDimensions=num2str(robot.numJoints));
demux = add_block("simulink/Signal Routing/Demux", importName + "/JointDemux");
set_param(demux, Outputs=num2str(robot.numJoints));
add_line(importName, "q/1", "JointDemux/1");

innerBlocks = [mechBlocks; {getfullname(demux)}];
for k = 1:robot.numJoints
    jointBlk = findJointBlock(importName, robot.jointNames(k));
    set_param(jointBlk, MotionActuationMode="InputMotion", TorqueActuationMode="ComputedTorque");

    conv = add_block("nesl_utility/Simulink-PS Converter", ...
        importName + "/q_" + robot.jointNames(k) + "_PS");
    % 二阶输入滤波仅用于可视化，提供运动驱动所需的一、二阶导数
    set_param(conv, Unit="rad", FilteringAndDerivatives="filter", ...
        SimscapeFilterOrder="2", InputFilterTimeConstant="0.01");
    add_line(importName, "JointDemux/" + k, get_param(conv, "Name") + "/1");

    convPorts = get_param(conv, "PortHandles");
    jointPorts = get_param(jointBlk, "PortHandles");
    add_line(importName, convPorts.RConn(1), freeMotionPort(jointPorts));
    innerBlocks{end+1} = getfullname(conv); %#ok<AGROW>
end

% 把除输入端口外的所有块包成子系统，再转换为 Subsystem Reference 文件
handles = cellfun(@(b) get_param(b, "Handle"), innerBlocks);
Simulink.BlockDiagram.createSubsystem(handles, Name=char(visualName));
subsysBlk = importName + "/" + visualName;
inner = find_system(subsysBlk, SearchDepth=1, BlockType="Inport");
set_param(inner{1}, Name="q");

prevDir = cd(modelsDir);
restoreDir = onCleanup(@() cd(prevDir));
Simulink.SubsystemReference.convertSubsystemToSubsystemReference(subsysBlk, char(visualName));
save_system(visualName);
close_system(importName, 0);
fprintf("已生成可视化子系统: %s\n", visualFile);
end

function closeIfOpen(name)
if bdIsLoaded(name)
    close_system(name, 0);
end
end

function fixMeshPaths(mdl, urdfFile)
solids = find_system(mdl, LookUnderMasks="all", FollowLinks="on", ...
    RegExp="on", ReferenceBlock=".*File Solid.*");
urdfDir = fileparts(urdfFile);
for k = 1:numel(solids)
    meshPath = string(get_param(solids{k}, "ExtGeomFileName"));
    resolved = resolveMesh(meshPath, urdfDir);
    if resolved == ""
        warning("build_robot_visual:meshNotFound", "找不到网格文件: %s", meshPath);
    else
        set_param(solids{k}, ExtGeomFileName=char(resolved));
    end
end
end

function resolved = resolveMesh(meshPath, urdfDir)
resolved = "";
rel = meshPath;
if startsWith(rel, "package://")
    % package://<pkg>/<rest>：在 URDF 所在目录及其上层逐级查找 <pkg> 或 <rest>
    parts = split(extractAfter(rel, "package://"), "/");
    candidates = [join(parts, "/"); join(parts(2:end), "/")];
else
    candidates = rel;
end
if isfile(rel)
    resolved = string(java.io.File(rel).getCanonicalPath());
    return
end
searchDir = urdfDir;
for level = 1:5
    for c = candidates.'
        p = fullfile(searchDir, c);
        if isfile(p)
            resolved = string(java.io.File(p).getCanonicalPath());
            return
        end
    end
    searchDir = fileparts(searchDir);
end
end

function blk = findJointBlock(mdl, jointName)
joints = find_system(mdl, LookUnderMasks="all", FollowLinks="on", ...
    RegExp="on", ReferenceBlock=".*Revolute Joint.*");
names = string(get_param(joints, "Name"));
idx = find(names == jointName | names == matlab.lang.makeValidName(jointName), 1);
if isempty(idx)
    error("build_robot_visual:jointNotFound", "Simscape 模型中找不到关节 '%s'。", jointName);
end
blk = joints{idx};
end

function port = freeMotionPort(jointPorts)
% 打开输入运动后新增的物理信号端口 q 是唯一尚未连线的连接端口
all = [jointPorts.LConn, jointPorts.RConn];
free = all(arrayfun(@(p) get_param(p, "Line") == -1, all));
if numel(free) ~= 1
    error("build_robot_visual:portAmbiguous", "无法唯一确定关节运动输入端口。");
end
port = free;
end
