// MATLAB 中直接调用 MINCO C 接口的 MEX 网关，便于离线实验，与 Simulink 使用同一份实现。
//
//   [ver, durations, totalTime] = minco_mex('plan', slot, dim, wpts, vMax, aMax, timeScale)
//       wpts: dim x numWpts（每列一个路径点），vMax/aMax: dim 元素向量
//   [pos, vel, acc, jerk, seg] = minco_mex('eval', slot, ver, t)
//       t: 任意长度时间向量；pos/vel/acc/jerk: dim x numel(t)

#include "mex.h"
#include "minco_c_api.h"

#include <cstring>
#include <string>

static int g_dim[MINCO_MAX_SLOTS] = {0};

static int scalarInt(const mxArray *a, const char *name)
{
    if (!mxIsNumeric(a) || mxGetNumberOfElements(a) != 1)
    {
        mexErrMsgIdAndTxt("minco_mex:arg", "%s 必须是数值标量", name);
    }
    return static_cast<int>(mxGetScalar(a));
}

static void plan(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    if (nrhs != 7)
    {
        mexErrMsgIdAndTxt("minco_mex:nargin", "用法: minco_mex('plan', slot, dim, wpts, vMax, aMax, timeScale)");
    }
    const int slot = scalarInt(prhs[1], "slot");
    const int dim = scalarInt(prhs[2], "dim");
    if (slot < 0 || slot >= MINCO_MAX_SLOTS || dim < 1 || dim > MINCO_MAX_DIM)
    {
        mexErrMsgIdAndTxt("minco_mex:arg", "slot 或 dim 越界");
    }
    const mxArray *W = prhs[3];
    if (!mxIsDouble(W) || static_cast<int>(mxGetM(W)) != dim)
    {
        mexErrMsgIdAndTxt("minco_mex:wpts", "wpts 必须是 dim x numWpts 的 double 矩阵");
    }
    const int numWpts = static_cast<int>(mxGetN(W));
    if (numWpts > MINCO_MAX_WPTS)
    {
        mexErrMsgIdAndTxt("minco_mex:wpts", "路径点数量不能超过 %d", MINCO_MAX_WPTS);
    }
    if (mxGetNumberOfElements(prhs[4]) < static_cast<size_t>(dim) ||
        mxGetNumberOfElements(prhs[5]) < static_cast<size_t>(dim))
    {
        mexErrMsgIdAndTxt("minco_mex:limits", "vMax/aMax 至少需要 dim 个元素");
    }

    double wpts[MINCO_MAX_DIM * MINCO_MAX_WPTS] = {0};
    const double *w = mxGetPr(W);
    for (int k = 0; k < numWpts; k++)
    {
        for (int d = 0; d < dim; d++)
        {
            wpts[d + MINCO_MAX_DIM * k] = w[d + dim * k];
        }
    }
    double durations[MINCO_MAX_WPTS - 1];
    double totalTime = 0.0;
    const int ver = minco_plan(slot, dim, numWpts, wpts, mxGetPr(prhs[4]), mxGetPr(prhs[5]),
                               mxGetScalar(prhs[6]), durations, &totalTime);
    if (ver > 0)
    {
        g_dim[slot] = dim;
    }

    plhs[0] = mxCreateDoubleScalar(ver);
    if (nlhs > 1)
    {
        plhs[1] = mxCreateDoubleMatrix(1, numWpts - 1, mxREAL);
        std::memcpy(mxGetPr(plhs[1]), durations, sizeof(double) * (numWpts - 1));
    }
    if (nlhs > 2)
    {
        plhs[2] = mxCreateDoubleScalar(totalTime);
    }
}

static void eval(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    if (nrhs != 4)
    {
        mexErrMsgIdAndTxt("minco_mex:nargin", "用法: minco_mex('eval', slot, ver, t)");
    }
    const int slot = scalarInt(prhs[1], "slot");
    const int ver = scalarInt(prhs[2], "ver");
    if (slot < 0 || slot >= MINCO_MAX_SLOTS || g_dim[slot] == 0)
    {
        mexErrMsgIdAndTxt("minco_mex:noplan", "该 slot 尚未成功规划");
    }
    const int dim = g_dim[slot];
    const size_t n = mxGetNumberOfElements(prhs[3]);
    const double *t = mxGetPr(prhs[3]);

    mxArray *out[5];
    for (int i = 0; i < 4; i++)
    {
        out[i] = mxCreateDoubleMatrix(dim, n, mxREAL);
    }
    out[4] = mxCreateDoubleMatrix(1, n, mxREAL);

    double p[MINCO_MAX_DIM], v[MINCO_MAX_DIM], a[MINCO_MAX_DIM], j[MINCO_MAX_DIM];
    for (size_t k = 0; k < n; k++)
    {
        const int seg = minco_eval(slot, ver, t[k], p, v, a, j);
        if (seg < 0)
        {
            mexErrMsgIdAndTxt("minco_mex:eval", "minco_eval 返回错误码 %d", seg);
        }
        for (int d = 0; d < dim; d++)
        {
            mxGetPr(out[0])[d + dim * k] = p[d];
            mxGetPr(out[1])[d + dim * k] = v[d];
            mxGetPr(out[2])[d + dim * k] = a[d];
            mxGetPr(out[3])[d + dim * k] = j[d];
        }
        mxGetPr(out[4])[k] = seg + 1;
    }
    for (int i = 0; i < 5; i++)
    {
        if (i < nlhs || i == 0)
        {
            plhs[i] = out[i];
        }
        else
        {
            mxDestroyArray(out[i]);
        }
    }
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    if (nrhs < 1 || !mxIsChar(prhs[0]))
    {
        mexErrMsgIdAndTxt("minco_mex:cmd", "第一个参数必须是字符向量 'plan' 或 'eval'（单引号，不支持字符串 \"plan\"）");
    }
    char buf[16];
    mxGetString(prhs[0], buf, sizeof(buf));
    const std::string cmd(buf);
    if (cmd == "plan")
    {
        plan(nlhs, plhs, nrhs, prhs);
    }
    else if (cmd == "eval")
    {
        eval(nlhs, plhs, nrhs, prhs);
    }
    else
    {
        mexErrMsgIdAndTxt("minco_mex:cmd", "未知命令 '%s'", buf);
    }
}
