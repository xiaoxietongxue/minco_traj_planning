function eigenDir = find_eigen(root)
%FIND_EIGEN 查找 Eigen 3 头文件目录（Simulink 自定义代码与 MEX 编译共用）。

candidates = [string(getenv("EIGEN3_INCLUDE_DIR")), ...
    fullfile(root, "third_party", "eigen"), ...
    "/opt/homebrew/include/eigen3", "/usr/local/include/eigen3", "/usr/include/eigen3"];
for c = candidates
    if c ~= "" && isfile(fullfile(c, "Eigen", "Core"))
        eigenDir = c;
        return
    end
end
error("find_eigen:notFound", ...
    "未找到 Eigen。请安装（brew install eigen）或设置环境变量 EIGEN3_INCLUDE_DIR。");
end
