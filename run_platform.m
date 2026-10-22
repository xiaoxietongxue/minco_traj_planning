function out = run_platform(opts)
%RUN_PLATFORM 一键运行：初始化 -> 仿真 -> 绘图。
%   out = run_platform                       使用 platform_config 中的规划模式
%   out = run_platform(PlanMode=2)           临时切换为笛卡尔空间规划
%   out = run_platform(Visualize=false)      不打开 Mechanics Explorer（批量仿真更快）

arguments
    opts.PlanMode double {mustBeMember(opts.PlanMode, [1 2])} = []
    opts.Visualize (1,1) logical = true
    opts.Plot (1,1) logical = true
end

startup_platform;
mdl = "RobotTrajPlatform";
cfg = evalin("base", "platformCfg");
robot = evalin("base", "platformRobot");
if ~isempty(opts.PlanMode)
    cfg.planMode = opts.PlanMode;
    assignin("base", "PlanMode", opts.PlanMode);
    assignin("base", "platformCfg", cfg);
end

open_system(mdl);
wasDirty = get_param(mdl, "Dirty");
set_param(mdl, SimMechanicsOpenEditorOnUpdate=char(matlab.lang.OnOffSwitchState(opts.Visualize)));
set_param(mdl, Dirty=wasDirty);

out = sim(mdl);
if opts.Plot
    plot_results(out, cfg, robot);
end
end
