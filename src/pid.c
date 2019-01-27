#include <pid.h>

uint32_t pid_scale(uint32_t val,
                   uint32_t input_min,
                   uint32_t input_max,
                   uint32_t output_min,
                   uint32_t output_max)
{
    uint32_t input_span = input_max - input_min;
    uint32_t output_span = output_max - output_min;

    return ((output_span * (val - input_min)) / input_span) + output_min;
}

pid_sat_dir_t pid_deadzone_limit(int32_t * val, uint32_t out_limit, uint32_t deadzone_limit)
{
    pid_sat_dir_t sat_dir = PID_SAT_NONE;

    int32_t val_cpy = *val;
    bool minus = ((uint32_t)val_cpy) & (1 << 31);
    if (minus)
    {
        val_cpy = -val_cpy;
    }

    if (val_cpy < deadzone_limit)
    {
        val_cpy = deadzone_limit;
        sat_dir = minus ? PID_SAT_LOWER : PID_SAT_UPPER;
    }
    else if (val_cpy > out_limit)
    {
        val_cpy = out_limit;
        sat_dir = minus ? PID_SAT_LOWER : PID_SAT_UPPER;
    }

    if (minus)
    {
        val_cpy = -val_cpy;
    }

    *val = val_cpy;
    return sat_dir;
}

pid_sat_dir_t pid_limit(int32_t * val, int32_t upper_lim, int32_t lower_lim)
{
    pid_sat_dir_t sat_dir = PID_SAT_NONE;
    if (*val > upper_lim)
    {
        *val = upper_lim;
        sat_dir = PID_SAT_UPPER;
    }
    else if (*val < lower_lim)
    {
        *val = lower_lim;
        sat_dir = PID_SAT_LOWER;
    }

    return sat_dir;
}

void pid_init(pid_t * pid, const pid_cfg_t * cfg)
{
    pid->kp_q = cfg->kp_q;
    pid->ki_q = cfg->ki_q;
    pid->kd_q = cfg->kd_q;
    pid->q    = cfg->q;

    pid->point_desired = 0;
    pid->last_point = 0;
    pid->integral_sum = 0;
}

void pid_point_set(pid_t * pid, int32_t point)
{
    pid->point_desired = point;
}

int32_t pid_proc(pid_t * pid, int32_t point_current)
{
    int32_t error = pid->point_desired - point_current;

    /* Anti wind-up. If error and saturation signs matches,
     * do not increase integral sum further. */
    if ((error * pid->saturation) <= 0)
    {
        pid->integral_sum += error * pid->ki_q;
    }

    int32_t p_part = error * pid->kp_q;
    int32_t i_part = pid->integral_sum;
    int32_t d_part = (point_current - pid->last_point) * pid->kd_q;

    int32_t output = p_part + i_part + d_part;
    output >>= pid->q;

    //pid->saturation = pid_limit(&output, PID_OUT_MAX, PID_OUT_MIN);
    //pid->saturation = pid_deadzone_limit(&output, PID_OUT_MAX, PID_OUT_DEADZONE_START);

    pid->last_point = point_current;

    return output;
}
