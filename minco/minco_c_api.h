#ifndef MINCO_C_API_H
#define MINCO_C_API_H

/*
 * MINCO (GCOPTER MINCO_S3NU, minimum-jerk) 的 C 接口，供 Simulink C Caller 调用。
 *
 * 数组约定（与 Simulink 列主序一致）：
 *   wpts      : MINCO_MAX_DIM x MINCO_MAX_WPTS，元素 (d,k) 位于 wpts[d + MINCO_MAX_DIM*k]，
 *               只使用前 dim 行、前 numWpts 列（含起点和终点）
 *   vMax/aMax : MINCO_MAX_DIM，只使用前 dim 个
 *   durations : MINCO_MAX_WPTS-1，未用部分置 0
 *   pos/vel/acc/jerk : MINCO_MAX_DIM，未用部分置 0
 *
 * 起止点速度、加速度均为零；分段时间按梯形速度启发式分配，
 * 再整体缩放到恰好满足各维 vMax/aMax，最后乘以 timeScale (>=1 留裕量)。
 */

#define MINCO_MAX_DIM 6
#define MINCO_MAX_WPTS 16
#define MINCO_MAX_SLOTS 4

#define MINCO_ERR_SLOT (-1)
#define MINCO_ERR_DIM (-2)
#define MINCO_ERR_WPTS (-3)
#define MINCO_ERR_LIMITS (-4)
#define MINCO_ERR_NUMERIC (-5)
#define MINCO_ERR_NOPLAN (-6)

#ifdef __cplusplus
extern "C" {
#endif

/* 返回 >0 的规划版本号；输入未变化时直接返回缓存版本号而不重新求解。返回 <0 为错误码。 */
int minco_plan(int slot, int dim, int numWpts, const double *wpts,
               const double *vMax, const double *aMax, double timeScale,
               double *durations, double *totalTime);

/* planVersion 为 minco_plan 的返回值，用于在 Simulink 中建立执行先后依赖。
   t 会被限制在 [0, totalTime]。返回当前所在段序号（从 0 开始），<0 为错误码。 */
int minco_eval(int slot, int planVersion, double t,
               double *pos, double *vel, double *acc, double *jerk);

#ifdef __cplusplus
}
#endif

#endif
