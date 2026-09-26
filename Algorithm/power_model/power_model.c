#include "power_model.h"
#include <stdbool.h>
#include <math.h>
float power_model_limit(const PowerModel *model, float raw[4], const float rpm[4], float budget, float *estimated)
{
    /* 对统一缩放系数 s：P(s)=A*s²+B*s+C。
     * A 为电阻损耗 ΣR*I²，B 为 Σ|Kt*I*ω|，C 为固定损耗估计。
     * 再生制动的负机械功率不拿来抵扣其他轮耗电，先采用保守上界。
     * 与旧代码不同：ω 来自实测 RPM 转 rad/s，不混用 motor_ref 和角速度。
     */
    float a = 0, b = 0;
    bool valid = isfinite(budget) && budget >= 0;
    for (unsigned i = 0; i < 4; ++i) {
        valid = valid && isfinite(raw[i]) && isfinite(rpm[i]);
        float amp = raw[i] * model->raw_to_amp;
        a += model->equivalent_resistance * amp * amp;
        b += fabsf(model->rotor_torque_constant * amp * rpm[i] * 0.104719755f);
    }
    float scale = 0;
    if (valid && isfinite(a) && isfinite(b) && budget > model->idle_w) {
        float available = budget - model->idle_w;
        if (a + b <= available) scale = 1;
        else if (a > 0.000001f) {
            /* 正根的等价写法，避免 -B + sqrt(...) 相消导致精度损失。 */
            scale = 2.0f * available / (b + sqrtf(b*b + 4.0f*a*available));
        } else if (b > 0) scale = available / b;
    }
    scale = fmaxf(0, fminf(scale, 1));
    for (unsigned i = 0; i < 4; ++i) raw[i] = valid ? raw[i] * scale : 0;
    *estimated = valid && isfinite(a) && isfinite(b) ?
        a*scale*scale + b*scale + model->idle_w : 0;
    return scale;
}

