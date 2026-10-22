function plot_results(out, cfg, robot)
%PLOT_RESULTS 绘制关节轨迹、限幅利用率与末端三维路径（含路径点和机器人位形）。

arguments
    out (1,1) Simulink.SimulationOutput
    cfg (1,1) struct
    robot (1,1) struct
end

t = out.log_q.Time;
q = toRows(out.log_q.Data, robot.numJoints);
qd = toRows(out.log_qd.Data, robot.numJoints);
qdd = toRows(out.log_qdd.Data, robot.numJoints);
ee = toRows(out.log_eePos.Data, 3);
ratio = toRows(out.log_limitRatio.Data, 3);
modeNames = ["关节空间", "笛卡尔空间"];
modeName = modeNames(cfg.planMode);

figure(Name="关节轨迹 - " + modeName);
tl = tiledlayout(3, 1, TileSpacing="compact");
title(tl, "MINCO 轨迹（" + modeName + "）");
plotJoint(t, rad2deg(q), "q [deg]");
plotJoint(t, rad2deg(qd), "qd [deg/s]", rad2deg(robot.vMax));
plotJoint(t, rad2deg(qdd), "qdd [deg/s^2]", rad2deg(robot.aMax));
xlabel(tl, "t [s]");
legend(nexttile(1), robot.jointNames, Interpreter="none", Location="eastoutside");

figure(Name="限幅利用率 - " + modeName);
plot(t, ratio(1, :), t, ratio(2, :), LineWidth=1.2);
yline(1, "r--", "限值");
grid on;
xlabel("t [s]");
ylabel("峰值 / 限值");
legend("速度", "加速度", Location="best");
title(sprintf("速度利用率峰值 %.2f，加速度利用率峰值 %.2f", max(ratio(1, :)), max(ratio(2, :))));

figure(Name="末端路径 - " + modeName);
ax = axes;
[~, kMid] = max(vecnorm(ee - ee(:, 1)));
for k = [1, kMid]
    show(robot.rbt, q(:, k).', Parent=ax, PreservePlot=true, Frames="off");
    hold(ax, "on");
end
set(findobj(ax, Type="patch"), FaceAlpha=0.18, EdgeColor="none");
plot3(ax, ee(1, :), ee(2, :), ee(3, :), "m-", LineWidth=2.5);
wp = waypointPositions(cfg, robot);
plot3(ax, wp(1, :), wp(2, :), wp(3, :), "ko", MarkerFaceColor="y", MarkerSize=9);
text(ax, wp(1, :), wp(2, :), wp(3, :) + 0.03, string(1:size(wp, 2)), FontWeight="bold");
hold(ax, "off");
axis(ax, "equal");
view(ax, 150, 30);
title(ax, "末端路径与路径点（品红：轨迹，黄点：路径点，半透明：起点与最远点位形）");
end

function plotJoint(t, Y, label, lim)
nexttile;
plot(t, Y, LineWidth=1.1);
grid on;
ylabel(label);
if nargin > 3
    hold on;
    yline(max(lim), "r--");
    yline(-max(lim), "r--");
    hold off;
end
end

function X = toRows(data, n)
X = squeeze(data);
if size(X, 1) ~= n
    X = X.';
end
end

function wp = waypointPositions(cfg, robot)
if cfg.planMode == 2
    wp = cfg.cart.waypoints(:, 1:3).';
    return
end
qW = deg2rad(cfg.joint.waypointsDeg);
wp = zeros(3, size(qW, 1));
for k = 1:size(qW, 1)
    T = getTransform(robot.rbt, qW(k, :), robot.eeName);
    wp(:, k) = T(1:3, 4);
end
end
