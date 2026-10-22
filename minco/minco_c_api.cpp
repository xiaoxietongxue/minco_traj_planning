#include "minco_c_api.h"

#include "gcopter/minco.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace
{
constexpr int kMaxPieces = MINCO_MAX_WPTS - 1;
constexpr int kNumCoeffs = 6;
constexpr int kSamplesPerPiece = 64;
constexpr double kMinPieceTime = 0.05;

struct PlanInput
{
    int dim;
    int numWpts;
    double wpts[MINCO_MAX_DIM * MINCO_MAX_WPTS];
    double vMax[MINCO_MAX_DIM];
    double aMax[MINCO_MAX_DIM];
    double timeScale;
};

struct PlanSlot
{
    bool valid;
    int version;
    PlanInput input;
    int dim;
    int numPieces;
    double T[kMaxPieces];
    double totalTime;
    // c[i][d][j]：第 i 段、第 d 维、t^j 的系数（升幂）
    double c[kMaxPieces][MINCO_MAX_DIM][kNumCoeffs];
};

PlanSlot g_slots[MINCO_MAX_SLOTS];

inline double wp(const PlanInput &in, int d, int k)
{
    return in.wpts[d + MINCO_MAX_DIM * k];
}

// 单维、单段的梯形速度曲线时间
double trapezoidTime(double dist, double v, double a)
{
    dist = std::fabs(dist);
    if (dist * a >= v * v)
    {
        return dist / v + v / a;
    }
    return 2.0 * std::sqrt(dist / a);
}

bool solve(const PlanInput &in, const double *T, PlanSlot &s)
{
    const int N = in.numWpts - 1;
    Eigen::VectorXd ts(N);
    for (int i = 0; i < N; i++)
    {
        ts(i) = T[i];
    }

    // GCOPTER 的 MINCO 固定为 3 维；各维相互解耦，因此按 3 维一组分批求解与整体求解等价
    for (int g = 0; g < in.dim; g += 3)
    {
        Eigen::Matrix3d head = Eigen::Matrix3d::Zero();
        Eigen::Matrix3d tail = Eigen::Matrix3d::Zero();
        Eigen::Matrix3Xd inner = Eigen::Matrix3Xd::Zero(3, N - 1);
        for (int r = 0; r < 3 && g + r < in.dim; r++)
        {
            head(r, 0) = wp(in, g + r, 0);
            tail(r, 0) = wp(in, g + r, N);
            for (int k = 0; k < N - 1; k++)
            {
                inner(r, k) = wp(in, g + r, k + 1);
            }
        }

        minco::MINCO_S3NU solver;
        solver.setConditions(head, tail, N);
        solver.setParameters(inner, ts);
        const Eigen::MatrixX3d &b = solver.getCoeffs();
        if (!b.allFinite())
        {
            return false;
        }
        for (int i = 0; i < N; i++)
        {
            for (int r = 0; r < 3 && g + r < in.dim; r++)
            {
                for (int j = 0; j < kNumCoeffs; j++)
                {
                    s.c[i][g + r][j] = b(kNumCoeffs * i + j, r);
                }
            }
        }
    }

    s.dim = in.dim;
    s.numPieces = N;
    s.totalTime = 0.0;
    for (int i = 0; i < N; i++)
    {
        s.T[i] = T[i];
        s.totalTime += T[i];
    }
    return true;
}

void evalPiece(const PlanSlot &s, int i, double t, int d,
               double &p, double &v, double &a, double &j)
{
    const double *c = s.c[i][d];
    p = c[0] + t * (c[1] + t * (c[2] + t * (c[3] + t * (c[4] + t * c[5]))));
    v = c[1] + t * (2.0 * c[2] + t * (3.0 * c[3] + t * (4.0 * c[4] + t * 5.0 * c[5])));
    a = 2.0 * c[2] + t * (6.0 * c[3] + t * (12.0 * c[4] + t * 20.0 * c[5]));
    j = 6.0 * c[3] + t * (24.0 * c[4] + t * 60.0 * c[5]);
}

// 返回 max_d( max|v_d|/vMax_d , sqrt(max|a_d|/aMax_d) )，即满足限幅所需的整体时间缩放倍数
double requiredTimeScale(const PlanSlot &s, const PlanInput &in)
{
    double k = 0.0;
    for (int i = 0; i < s.numPieces; i++)
    {
        for (int n = 0; n <= kSamplesPerPiece; n++)
        {
            const double t = s.T[i] * n / kSamplesPerPiece;
            for (int d = 0; d < s.dim; d++)
            {
                double p, v, a, j;
                evalPiece(s, i, t, d, p, v, a, j);
                k = std::max(k, std::fabs(v) / in.vMax[d]);
                k = std::max(k, std::sqrt(std::fabs(a) / in.aMax[d]));
            }
        }
    }
    return k;
}

bool sameInput(const PlanInput &a, const PlanInput &b)
{
    return std::memcmp(&a, &b, sizeof(PlanInput)) == 0;
}
} // namespace

extern "C" int minco_plan(int slot, int dim, int numWpts, const double *wpts,
                          const double *vMax, const double *aMax, double timeScale,
                          double *durations, double *totalTime)
{
    if (slot < 0 || slot >= MINCO_MAX_SLOTS)
    {
        return MINCO_ERR_SLOT;
    }
    if (dim < 1 || dim > MINCO_MAX_DIM)
    {
        return MINCO_ERR_DIM;
    }
    if (numWpts < 2 || numWpts > MINCO_MAX_WPTS)
    {
        return MINCO_ERR_WPTS;
    }

    PlanInput in;
    std::memset(&in, 0, sizeof(in));
    in.dim = dim;
    in.numWpts = numWpts;
    for (int k = 0; k < numWpts; k++)
    {
        for (int d = 0; d < dim; d++)
        {
            in.wpts[d + MINCO_MAX_DIM * k] = wpts[d + MINCO_MAX_DIM * k];
        }
    }
    for (int d = 0; d < dim; d++)
    {
        if (!(vMax[d] > 0.0) || !(aMax[d] > 0.0))
        {
            return MINCO_ERR_LIMITS;
        }
        in.vMax[d] = vMax[d];
        in.aMax[d] = aMax[d];
    }
    // timeScale < 1 会破坏限幅保证
    in.timeScale = timeScale > 1.0 ? timeScale : 1.0;

    PlanSlot &s = g_slots[slot];
    if (!(s.valid && sameInput(s.input, in)))
    {
        const int N = numWpts - 1;
        double T[kMaxPieces];
        for (int i = 0; i < N; i++)
        {
            double Ti = kMinPieceTime;
            for (int d = 0; d < dim; d++)
            {
                Ti = std::max(Ti, trapezoidTime(wp(in, d, i + 1) - wp(in, d, i),
                                                in.vMax[d], in.aMax[d]));
            }
            T[i] = Ti;
        }

        s.valid = false;
        if (!solve(in, T, s))
        {
            return MINCO_ERR_NUMERIC;
        }

        // 起止速度/加速度为零时，最小 jerk 解对整体时间缩放是等变的，一次缩放即可贴合限幅
        const double k = requiredTimeScale(s, in);
        if (k > 1e-9)
        {
            for (int i = 0; i < N; i++)
            {
                T[i] = std::max(T[i] * k * in.timeScale, kMinPieceTime);
            }
            if (!solve(in, T, s))
            {
                return MINCO_ERR_NUMERIC;
            }
        }

        s.input = in;
        s.valid = true;
        s.version = s.version % 1000000 + 1;
    }

    for (int i = 0; i < kMaxPieces; i++)
    {
        durations[i] = i < s.numPieces ? s.T[i] : 0.0;
    }
    *totalTime = s.totalTime;
    return s.version;
}

extern "C" int minco_eval(int slot, int planVersion, double t,
                          double *pos, double *vel, double *acc, double *jerk)
{
    for (int d = 0; d < MINCO_MAX_DIM; d++)
    {
        pos[d] = vel[d] = acc[d] = jerk[d] = 0.0;
    }
    if (slot < 0 || slot >= MINCO_MAX_SLOTS)
    {
        return MINCO_ERR_SLOT;
    }
    const PlanSlot &s = g_slots[slot];
    if (planVersion <= 0 || !s.valid)
    {
        return MINCO_ERR_NOPLAN;
    }

    t = std::min(std::max(t, 0.0), s.totalTime);
    int i = 0;
    while (i < s.numPieces - 1 && t > s.T[i])
    {
        t -= s.T[i];
        i++;
    }
    t = std::min(t, s.T[i]);

    for (int d = 0; d < s.dim; d++)
    {
        evalPiece(s, i, t, d, pos[d], vel[d], acc[d], jerk[d]);
    }
    return i;
}
