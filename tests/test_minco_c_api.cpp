#include "minco_c_api.h"

#include <cmath>
#include <cstdio>
#include <cstring>

static int g_failures = 0;

#define CHECK(cond, ...)                      \
    do                                        \
    {                                         \
        if (!(cond))                          \
        {                                     \
            std::printf("FAIL: " __VA_ARGS__); \
            std::printf("\n");                \
            g_failures++;                     \
        }                                     \
    } while (0)

static double W[MINCO_MAX_DIM * MINCO_MAX_WPTS];

static void setWp(int d, int k, double v) { W[d + MINCO_MAX_DIM * k] = v; }

static void testSixDofJointPlan()
{
    const int dim = 6, n = 4;
    std::memset(W, 0, sizeof(W));
    const double q[4][6] = {{0, -1.57, 0, -1.57, 0, 0},
                            {0.8, -1.2, 0.9, -1.2, -0.6, 0.5},
                            {-0.5, -1.0, 1.4, -2.0, 0.7, -1.0},
                            {0.2, -1.8, 0.4, -1.0, 0.0, 1.2}};
    for (int k = 0; k < n; k++)
        for (int d = 0; d < dim; d++)
            setWp(d, k, q[k][d]);
    double vMax[6] = {3.14, 3.14, 3.14, 3.14, 3.14, 3.14};
    double aMax[6] = {4, 4, 4, 6, 6, 6};
    double durations[MINCO_MAX_WPTS - 1], total = 0;

    const int ver = minco_plan(0, dim, n, W, vMax, aMax, 1.0, durations, &total);
    CHECK(ver > 0, "plan returned %d", ver);
    CHECK(total > 0, "total time %g", total);
    std::printf("6-DOF plan: version %d, total %.4f s, T = [%.3f %.3f %.3f]\n",
                ver, total, durations[0], durations[1], durations[2]);

    double pos[6], vel[6], acc[6], jer[6];
    double tk = 0;
    for (int k = 0; k < n; k++)
    {
        minco_eval(0, ver, tk, pos, vel, acc, jer);
        for (int d = 0; d < dim; d++)
            CHECK(std::fabs(pos[d] - q[k][d]) < 1e-9, "waypoint %d dim %d err %g", k, d, pos[d] - q[k][d]);
        if (k == 0 || k == n - 1)
            for (int d = 0; d < dim; d++)
                CHECK(std::fabs(vel[d]) < 1e-9 && std::fabs(acc[d]) < 1e-9, "boundary %d dim %d not at rest", k, d);
        if (k < n - 1)
            tk += durations[k];
    }

    // 连接处两侧 C2 连续，全程不超限（容差 1%，来自采样检查）
    double peakV = 0, peakA = 0;
    const int samples = 20000;
    double prev[6] = {0};
    for (int s = 0; s <= samples; s++)
    {
        const double t = total * s / samples;
        minco_eval(0, ver, t, pos, vel, acc, jer);
        for (int d = 0; d < dim; d++)
        {
            peakV = std::fmax(peakV, std::fabs(vel[d]) / vMax[d]);
            peakA = std::fmax(peakA, std::fabs(acc[d]) / aMax[d]);
            if (s > 0)
                CHECK(std::fabs(acc[d] - prev[d]) < 0.05, "acc jump at t=%g dim %d", t, d);
            prev[d] = acc[d];
        }
    }
    std::printf("peak vel ratio %.4f, peak acc ratio %.4f\n", peakV, peakA);
    CHECK(peakV <= 1.01 && peakA <= 1.01, "limits exceeded");
    CHECK(peakV > 0.98 || peakA > 0.98, "time scaling not tight");

    const int ver2 = minco_plan(0, dim, n, W, vMax, aMax, 1.0, durations, &total);
    CHECK(ver2 == ver, "cached plan should keep version (%d vs %d)", ver2, ver);

    setWp(0, 3, 0.3);
    const int ver3 = minco_plan(0, dim, n, W, vMax, aMax, 1.0, durations, &total);
    CHECK(ver3 == ver + 1, "changed input should replan (%d)", ver3);

    const double before = total;
    minco_plan(0, dim, n, W, vMax, aMax, 2.0, durations, &total);
    CHECK(std::fabs(total - 2.0 * before) < 1e-9, "timeScale 2 should double duration");
}

static void testErrors()
{
    double v[6] = {1, 1, 1, 1, 1, 1}, a[6] = {1, 1, 1, 1, 1, 1}, dur[MINCO_MAX_WPTS - 1], tot;
    CHECK(minco_plan(9, 3, 3, W, v, a, 1, dur, &tot) == MINCO_ERR_SLOT, "slot");
    CHECK(minco_plan(1, 7, 3, W, v, a, 1, dur, &tot) == MINCO_ERR_DIM, "dim");
    CHECK(minco_plan(1, 3, 1, W, v, a, 1, dur, &tot) == MINCO_ERR_WPTS, "wpts");
    v[1] = 0;
    CHECK(minco_plan(1, 3, 3, W, v, a, 1, dur, &tot) == MINCO_ERR_LIMITS, "limits");
    double p[6], vel[6], acc[6], j[6];
    CHECK(minco_eval(3, 1, 0, p, vel, acc, j) == MINCO_ERR_NOPLAN, "noplan");
}

int main()
{
    testSixDofJointPlan();
    testErrors();
    std::printf(g_failures ? "%d FAILURE(S)\n" : "ALL PASSED\n", g_failures);
    return g_failures ? 1 : 0;
}
