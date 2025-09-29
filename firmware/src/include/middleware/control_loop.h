#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#ifdef __cplusplus
extern "C"
{
#endif

    void control_loop_init(void);
    void control_loop_update(void);
    void control_loop_set_motor_setpoints(const double* setpoints);
    void control_loop_set_motor_speeds_int(int fl, int fr, int rl, int rr);

#ifdef __cplusplus
}
#endif

#endif // CONTROL_LOOP_H