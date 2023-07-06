/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.6-dev */

#ifndef PB_DRV_COMMS_CONFIG_PB_H_INCLUDED
#define PB_DRV_COMMS_CONFIG_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Struct definitions */
typedef struct _Config_BalancingConfig { 
    float balance_expo; 
    float balance_angle_scaling; 
    int32_t max_start_angle_steer; 
    int32_t shutoff_angle_steer; 
    int32_t shutoff_angle_drive; 
    int32_t max_update_limiter; 
    float output_lpf_rc; 
    int32_t balance_d_param_limiter; 
    float balance_d_param_lpf_rc; 
    uint32_t global_gyro_lpf; 
    bool has_imu_beta;
    float imu_beta; 
    bool has_expo_type;
    int32_t expo_type; 
    bool has_pid_to_current_mult;
    float pid_to_current_mult; 
    bool has_angle_to_rate_mult;
    float angle_to_rate_mult; 
} Config_BalancingConfig;

typedef struct _Config_Callibration { 
    float x_offset; 
    float y_offset; 
    float z_offset; 
} Config_Callibration;

typedef struct _Config_FootPadSettings { 
    float filter_rc; 
    int32_t min_level_to_start; 
    int32_t min_level_to_continue; 
    int32_t shutoff_delay_ms; 
} Config_FootPadSettings;

typedef struct _Config_PidConfig { 
    float p; 
    float d; 
    float i; 
    float max_i; 
    bool has_i_expo;
    float i_expo; 
} Config_PidConfig;

typedef struct _Config { 
    bool has_callibration;
    Config_Callibration callibration; 
    Config_PidConfig roll_angle_pid; 
    Config_FootPadSettings foot_pad; 
    Config_BalancingConfig balance_settings; 
    Config_PidConfig yaw_pid; 
    Config_PidConfig roll_rate_pid; 
    Config_PidConfig pitch_rate_pid; 
    Config_PidConfig pitch_angle_pid; 
} Config;


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define Config_init_default                      {false, Config_Callibration_init_default, Config_PidConfig_init_default, Config_FootPadSettings_init_default, Config_BalancingConfig_init_default, Config_PidConfig_init_default, Config_PidConfig_init_default, Config_PidConfig_init_default, Config_PidConfig_init_default}
#define Config_Callibration_init_default         {0.0f, 0.0f, 0.0f}
#define Config_PidConfig_init_default            {0.0f, 0.0f, 0.0f, 1.0f, false, 0.0f}
#define Config_FootPadSettings_init_default      {0.05f, 3300, 2000, 100}
#define Config_BalancingConfig_init_default      {0.15f, 15.0f, 15, 40, 14, 300, 1.0f, 300, 0.15f, 2u, false, 0.02f, false, 0, false, 0.0f, false, 0.0f}
#define Config_init_zero                         {false, Config_Callibration_init_zero, Config_PidConfig_init_zero, Config_FootPadSettings_init_zero, Config_BalancingConfig_init_zero, Config_PidConfig_init_zero, Config_PidConfig_init_zero, Config_PidConfig_init_zero, Config_PidConfig_init_zero}
#define Config_Callibration_init_zero            {0, 0, 0}
#define Config_PidConfig_init_zero               {0, 0, 0, 0, false, 0}
#define Config_FootPadSettings_init_zero         {0, 0, 0, 0}
#define Config_BalancingConfig_init_zero         {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, false, 0, false, 0, false, 0, false, 0}

/* Field tags (for use in manual encoding/decoding) */
#define Config_BalancingConfig_balance_expo_tag  1
#define Config_BalancingConfig_balance_angle_scaling_tag 2
#define Config_BalancingConfig_max_start_angle_steer_tag 3
#define Config_BalancingConfig_shutoff_angle_steer_tag 4
#define Config_BalancingConfig_shutoff_angle_drive_tag 5
#define Config_BalancingConfig_max_update_limiter_tag 6
#define Config_BalancingConfig_output_lpf_rc_tag 7
#define Config_BalancingConfig_balance_d_param_limiter_tag 8
#define Config_BalancingConfig_balance_d_param_lpf_rc_tag 9
#define Config_BalancingConfig_global_gyro_lpf_tag 10
#define Config_BalancingConfig_imu_beta_tag      11
#define Config_BalancingConfig_expo_type_tag     12
#define Config_BalancingConfig_pid_to_current_mult_tag 13
#define Config_BalancingConfig_angle_to_rate_mult_tag 14
#define Config_Callibration_x_offset_tag         4
#define Config_Callibration_y_offset_tag         5
#define Config_Callibration_z_offset_tag         6
#define Config_FootPadSettings_filter_rc_tag     1
#define Config_FootPadSettings_min_level_to_start_tag 2
#define Config_FootPadSettings_min_level_to_continue_tag 3
#define Config_FootPadSettings_shutoff_delay_ms_tag 4
#define Config_PidConfig_p_tag                   1
#define Config_PidConfig_d_tag                   2
#define Config_PidConfig_i_tag                   3
#define Config_PidConfig_max_i_tag               4
#define Config_PidConfig_i_expo_tag              13
#define Config_callibration_tag                  1
#define Config_roll_angle_pid_tag                2
#define Config_foot_pad_tag                      3
#define Config_balance_settings_tag              4
#define Config_yaw_pid_tag                       8
#define Config_roll_rate_pid_tag                 9
#define Config_pitch_rate_pid_tag                14
#define Config_pitch_angle_pid_tag               15

/* Struct field encoding specification for nanopb */
#define Config_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  callibration,      1) \
X(a, STATIC,   REQUIRED, MESSAGE,  roll_angle_pid,    2) \
X(a, STATIC,   REQUIRED, MESSAGE,  foot_pad,          3) \
X(a, STATIC,   REQUIRED, MESSAGE,  balance_settings,   4) \
X(a, STATIC,   REQUIRED, MESSAGE,  yaw_pid,           8) \
X(a, STATIC,   REQUIRED, MESSAGE,  roll_rate_pid,     9) \
X(a, STATIC,   REQUIRED, MESSAGE,  pitch_rate_pid,   14) \
X(a, STATIC,   REQUIRED, MESSAGE,  pitch_angle_pid,  15)
#define Config_CALLBACK NULL
#define Config_DEFAULT NULL
#define Config_callibration_MSGTYPE Config_Callibration
#define Config_roll_angle_pid_MSGTYPE Config_PidConfig
#define Config_foot_pad_MSGTYPE Config_FootPadSettings
#define Config_balance_settings_MSGTYPE Config_BalancingConfig
#define Config_yaw_pid_MSGTYPE Config_PidConfig
#define Config_roll_rate_pid_MSGTYPE Config_PidConfig
#define Config_pitch_rate_pid_MSGTYPE Config_PidConfig
#define Config_pitch_angle_pid_MSGTYPE Config_PidConfig

#define Config_Callibration_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, FLOAT,    x_offset,          4) \
X(a, STATIC,   REQUIRED, FLOAT,    y_offset,          5) \
X(a, STATIC,   REQUIRED, FLOAT,    z_offset,          6)
#define Config_Callibration_CALLBACK NULL
#define Config_Callibration_DEFAULT (const pb_byte_t*)"\x25\x00\x00\x00\x00\x2d\x00\x00\x00\x00\x35\x00\x00\x00\x00\x00"

#define Config_PidConfig_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, FLOAT,    p,                 1) \
X(a, STATIC,   REQUIRED, FLOAT,    d,                 2) \
X(a, STATIC,   REQUIRED, FLOAT,    i,                 3) \
X(a, STATIC,   REQUIRED, FLOAT,    max_i,             4) \
X(a, STATIC,   OPTIONAL, FLOAT,    i_expo,           13)
#define Config_PidConfig_CALLBACK NULL
#define Config_PidConfig_DEFAULT (const pb_byte_t*)"\x0d\x00\x00\x00\x00\x15\x00\x00\x00\x00\x1d\x00\x00\x00\x00\x25\x00\x00\x80\x3f\x6d\x00\x00\x00\x00\x00"

#define Config_FootPadSettings_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, FLOAT,    filter_rc,         1) \
X(a, STATIC,   REQUIRED, INT32,    min_level_to_start,   2) \
X(a, STATIC,   REQUIRED, INT32,    min_level_to_continue,   3) \
X(a, STATIC,   REQUIRED, INT32,    shutoff_delay_ms,   4)
#define Config_FootPadSettings_CALLBACK NULL
#define Config_FootPadSettings_DEFAULT (const pb_byte_t*)"\x0d\xcd\xcc\x4c\x3d\x10\xe4\x19\x18\xd0\x0f\x20\x64\x00"

#define Config_BalancingConfig_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, FLOAT,    balance_expo,      1) \
X(a, STATIC,   REQUIRED, FLOAT,    balance_angle_scaling,   2) \
X(a, STATIC,   REQUIRED, INT32,    max_start_angle_steer,   3) \
X(a, STATIC,   REQUIRED, INT32,    shutoff_angle_steer,   4) \
X(a, STATIC,   REQUIRED, INT32,    shutoff_angle_drive,   5) \
X(a, STATIC,   REQUIRED, INT32,    max_update_limiter,   6) \
X(a, STATIC,   REQUIRED, FLOAT,    output_lpf_rc,     7) \
X(a, STATIC,   REQUIRED, INT32,    balance_d_param_limiter,   8) \
X(a, STATIC,   REQUIRED, FLOAT,    balance_d_param_lpf_rc,   9) \
X(a, STATIC,   REQUIRED, UINT32,   global_gyro_lpf,  10) \
X(a, STATIC,   OPTIONAL, FLOAT,    imu_beta,         11) \
X(a, STATIC,   OPTIONAL, INT32,    expo_type,        12) \
X(a, STATIC,   OPTIONAL, FLOAT,    pid_to_current_mult,  13) \
X(a, STATIC,   OPTIONAL, FLOAT,    angle_to_rate_mult,  14)
#define Config_BalancingConfig_CALLBACK NULL
#define Config_BalancingConfig_DEFAULT (const pb_byte_t*)"\x0d\x9a\x99\x19\x3e\x15\x00\x00\x70\x41\x18\x0f\x20\x28\x28\x0e\x30\xac\x02\x3d\x00\x00\x80\x3f\x40\xac\x02\x4d\x9a\x99\x19\x3e\x50\x02\x5d\x0a\xd7\xa3\x3c\x60\x00\x6d\x00\x00\x00\x00\x75\x00\x00\x00\x00\x00"

extern const pb_msgdesc_t Config_msg;
extern const pb_msgdesc_t Config_Callibration_msg;
extern const pb_msgdesc_t Config_PidConfig_msg;
extern const pb_msgdesc_t Config_FootPadSettings_msg;
extern const pb_msgdesc_t Config_BalancingConfig_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define Config_fields &Config_msg
#define Config_Callibration_fields &Config_Callibration_msg
#define Config_PidConfig_fields &Config_PidConfig_msg
#define Config_FootPadSettings_fields &Config_FootPadSettings_msg
#define Config_BalancingConfig_fields &Config_BalancingConfig_msg

/* Maximum encoded size of messages (where known) */
#define Config_BalancingConfig_size              107
#define Config_Callibration_size                 15
#define Config_FootPadSettings_size              38
#define Config_PidConfig_size                    25
#define Config_size                              301

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
