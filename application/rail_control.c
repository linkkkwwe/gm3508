#include "rail_control.h"
#include "main.h"
#include "trajectory.h"
#include "Motor.h"

/* ==================== 可调参数（占位值，实车按实际行程改） ==================== */

/* 零点固定 = 上电位置，全程不再平移。
 * 摇杆回中**不会**重设零点了（原先每停一次就把当前位置设成 0°，
 * 结果 ±1080 限位跟着一起平移，等于没有绝对行程保护）。
 * 现在 0° 就是上电那一刻的编码器位置，限位是真正的绝对行程限位。 */

/* 手动档也走限位：目标角度累加到 ±1080° 就夹住，撞不到机械端点。
 * 这里除了依赖 Motor.c 的夹紧，还要**在本文件里同时夹住 manual_target**
 * ——manual_target 是本地累加器，如果只靠 Motor.c 夹，累加器会一路涨到
 * 几千度，用户往回推时要先把虚高的数字降回来，手感就是"反向一段空行程"。 */

/* 摇杆死区：|stick| 不超过这个值，目标完全不动（前馈也归零）。
 * 遥控器 ADC 中位有几个 count 的噪声，必须留死区，否则松开摇杆电机也会慢慢爬。
 * 摇杆中位噪声大、松开还在动，就把这个值往上加。 */
#define RAIL_STICK_DEADZONE      30

/* 摇杆满推（±1000）对应的移动速度（度/秒）。
 * 参考：三角波匀速段现在是 1200°/s（900°/3s），正弦峰值 1885°/s。这个值太小的话手动档会明显
 * "又慢又软"——低速落在静摩擦主导区，没动量，走一下卡一下。
 * 想和波形一样快就设 700 左右；想留点精细控制余量设 300~500。 */
#define RAIL_STICK_SEN          700.0f

/* 归位速度（度/秒）：从当前位置平滑走回 0° 的速度 */
#define RAIL_HOMING_SPEED        300.0f

/* 链路超时（ms）：超过这个时间没收到合法帧 → SAFE。
 * 取 200ms 而不是 100ms，因为无线链路会偶发抖动，阈值太小会误停。 */
#define RAIL_LINK_TIMEOUT_MS     200U

/* ===== 水平轴拨杆（s[0]，物理右侧）的档位原始值 =====
 * 标准编码：上=1、中=3、下=2（中档是 3 不是 2）。
 * 功能分配：上=波形1，中=波形2，下=摇杆手动。 */
#define RC_SW_POS_UP    1
#define RC_SW_POS_DOWN  2

/* ==================== 接收缓冲 ==================== */

#define RAIL_RX_BUF_SIZE 24U

static uint8_t  rx_buf[RAIL_RX_BUF_SIZE];
static volatile uint8_t rx_len;
static volatile uint8_t line_ready;

/* ==================== 状态 ==================== */

typedef enum
{
    RAIL_WAVE1 = 0,      /* 正弦波形 */
    RAIL_WAVE2,          /* 三角波形 */
    RAIL_MANUAL,         /* 摇杆速度控制 */
    RAIL_HOMING,         /* 归位到 0° 的过渡态 */
    RAIL_SAFE            /* 链路异常：停在原地 */
} rail_mode_e;

static rail_mode_e mode;
static rail_mode_e pending_wave;      /* HOMING 结束后要进哪个波形 */
static fp32 manual_target;            /* 手动模式的位置累加值 */
static fp32 homing_target;            /* 归位过程的中间目标 */
static int32_t stick_input;           /* 最近一帧的摇杆值 -1000~1000 */
static uint32_t last_frame_tick;
static uint8_t  frame_received;
static uint8_t  manual_fresh;         /* 1 = 刚进手动，需用当前位置起头 */
static uint8_t  homing_fresh;         /* 1 = 刚进归位，需用当前位置起头 */

/* ==================== 调试观测变量 ====================
 * 上面的 static 变量在 Keil 里不好 Watch（名字也容易和别的文件重名），
 * 这里照 Motor.c 的 g_dbg_* 风格导出一份。加 volatile 防止被优化掉。
 * 排障顺序看这两个计数器：
 *   g_dbg_rc_bytes  不涨 → 串口一个字节都没收到（接线/蓝牙/波特率）
 *   g_dbg_rc_bytes 涨、g_dbg_rc_frames 不涨 → 收到字节但帧解析失败（格式/帧头/校验）
 *   g_dbg_rc_frames 涨、电机不动 → 帧没问题，看 g_dbg_rc_mode 和电机侧 */
volatile uint32_t g_dbg_rc_bytes;     /**< 累计收到的字节数 */
volatile uint32_t g_dbg_rc_frames;    /**< 累计解析成功的帧数 */
volatile uint8_t  g_dbg_rc_mode;      /**< 当前模式：0=波形1 1=波形2 2=手动 3=归位 4=SAFE */
volatile int16_t  g_dbg_rc_stick;     /**< 最近一帧的摇杆值 */
volatile fp32     g_dbg_rc_target;    /**< 状态机输出的目标角度（度） */
volatile uint8_t  g_dbg_rc_ready;     /**< line_ready 标志（1 = 收到一整行待处理） */

/* ==================== 解析 ==================== */

/**
 * @brief 解析一个可选带负号的十进制整数，成功返回 1 并把指针推到数字之后。
 */
static uint8_t parse_int(const char **pp, int32_t *out)
{
    const char *p = *pp;
    int32_t sign = 1;
    int32_t v = 0;
    uint8_t digits = 0U;

    if (*p == '-')
    {
        sign = -1;
        p++;
    }

    while (*p >= '0' && *p <= '9')
    {
        v = v * 10 + (*p - '0');
        if (v > 100000)               /* 防溢出：越界值靠后面的范围校验拦掉 */
            v = 100000;
        p++;
        digits++;
    }

    if (!digits)
        return 0U;

    *out = sign * v;
    *pp = p;
    return 1U;
}

/**
 * @brief 解析一整行 "$B,<sw>,<stick>,<en>"。
 *        "$B" 帧头 + 字段范围校验足够自同步：串口助手灌帧、串扰的遥测行
 *        都能被正确丢弃，不会解出乱数据。
 */
static uint8_t parse_frame(const char *s, int32_t *sw, int32_t *stick, int32_t *en)
{
    if (s[0] != '$' || s[1] != 'B' || s[2] != ',')
        return 0U;
    s += 3;

    if (!parse_int(&s, sw) || (*s++ != ','))
        return 0U;
    if (!parse_int(&s, stick) || (*s++ != ','))
        return 0U;
    if (!parse_int(&s, en))
        return 0U;

    /* 范围校验：任何一项越界都当整帧非法丢弃（档位 0 也视为无效） */
    if (*sw < 1 || *sw > 3)
        return 0U;
    if (*stick < -1000 || *stick > 1000)
        return 0U;
    if (*en != 0 && *en != 1)
        return 0U;

    return 1U;
}

/* ==================== 接收 ==================== */

void rail_control_rx_byte(uint8_t byte)
{
    g_dbg_rc_bytes++;                 /* 排障用：这个不涨说明一个字节都没收到 */

    if (byte == '\r' || byte == '\n')
    {
        if (rx_len > 0U && !line_ready)
            line_ready = 1U;          /* 有内容才算一行 */
    }
    else if (!line_ready)
    {
        /* 缓冲满：回卷覆盖，让缓冲里放的始终是"最近"的字节。
         * 原来的写法是满了就丢弃新字节，缓冲会一直钉在最早的 23 个字节上，
         * 此后即使收到完整帧也进不来，只能等某个 \r\n 触发一次失败解析才腾空。
         * 注意：这条只是健壮性改进，帧末尾的 \r\n 仍能自愈，
         * 单靠它治不了链路断连。 */
        if (rx_len >= RAIL_RX_BUF_SIZE - 1U)
            rx_len = 0U;
        rx_buf[rx_len] = byte;
        rx_len++;
    }
    /* 上一行还没处理：多余字节丢弃 */

    g_dbg_rc_ready = line_ready;
}

/* ==================== 状态机 ==================== */

void rail_control_init(void)
{
    mode = RAIL_SAFE;
    pending_wave = RAIL_WAVE1;
    manual_target = 0.0f;
    homing_target = 0.0f;
    stick_input = 0;
    last_frame_tick = 0U;
    frame_received = 0U;
    manual_fresh = 0U;
    homing_fresh = 0U;
    rx_len = 0U;
    line_ready = 0U;
}

/**
 * @brief 处理一行完整数据，更新模式与输入。
 */
static void handle_line(void)
{
    char line[RAIL_RX_BUF_SIZE];
    int32_t sw, stick, en;
    uint8_t n = rx_len;
    uint8_t i;

    for (i = 0U; i < n; i++)
        line[i] = (char)rx_buf[i];
    line[n] = '\0';

    /* 先清接收状态：标志没清前中断不会写入新数据 */
    rx_len = 0U;
    line_ready = 0U;
    g_dbg_rc_ready = 0U;

    if (!parse_frame(line, &sw, &stick, &en))
        return;

    g_dbg_rc_frames++;                /* 排障用：帧解析成功计数 */
    g_dbg_rc_stick = (int16_t)stick;

    last_frame_tick = HAL_GetTick();
    frame_received = 1U;
    stick_input = stick;

    if (en == 0)
    {
        mode = RAIL_SAFE;
    }
    else if (sw == RC_SW_POS_DOWN)     /* 下档 = 摇杆手动 */
    {
        if (mode != RAIL_MANUAL)
            manual_fresh = 1U;        /* 用当前角度起头，避免目标跳变 */
        mode = RAIL_MANUAL;
    }
    else
    {
        /* 上档 = 波形1（正弦），中档 = 波形2（三角） */
        pending_wave = (sw == RC_SW_POS_UP) ? RAIL_WAVE1 : RAIL_WAVE2;
        if (mode != pending_wave)
        {
            if (mode != RAIL_HOMING)
                homing_fresh = 1U;    /* 用当前角度起头 */
            mode = RAIL_HOMING;
        }
    }
}

/**
 * @brief 手动模式：摇杆当速度旋钮，位置在本地累加。
 */
static fp32 manual_step(fp32 current_angle)
{
    fp32 step;

    if (manual_fresh)
    {
        manual_target = current_angle;   /* 从当前位置接管，无跳变 */
        manual_fresh = 0U;
    }

    /* 摇杆死区：中位就完全不动。
     * ⚠️ 这里必须"真的归零"而不只是"判定停下"——零点已固定为上电位置、
     *    不再重设，如果死区内还按摇杆原始值累加，遥控器 ADC 中位那几个
     *    count 的噪声会让 manual_target 一直漂，电机就慢慢爬走。
     *    顺带前馈也归零，否则摩擦补偿会顶着不动的地方继续推。 */
    if ((stick_input <= RAIL_STICK_DEADZONE) && (stick_input >= -RAIL_STICK_DEADZONE))
    {
        g_horizontal_ff = 0.0f;
        return manual_target;
    }

    step = ((fp32)stick_input / 1000.0f) * RAIL_STICK_SEN * 0.001f;

    /* 摩擦前馈：按目标运动方向给一个固定偏置电流，跨过静摩擦死区，
     * 治低速爬行。能走到这里说明不在死区内，step 必不为 0。 */
    if (step > 0.0f)
        g_horizontal_ff = g_horizontal_ff_gain;
    else
        g_horizontal_ff = -g_horizontal_ff_gain;

    manual_target += step;

    /* 本地累加器也要夹到 ±1080，不能只靠 Motor.c 夹目标角度。
     * 否则顶到限位后累加器还在虚涨，往回推要先降掉这段虚高值，
     * 表现出来就是反向时的"空行程"。 */
    if (manual_target > HORIZONTAL_MAX_ANGLE_DEG)
        manual_target = HORIZONTAL_MAX_ANGLE_DEG;
    else if (manual_target < HORIZONTAL_MIN_ANGLE_DEG)
        manual_target = HORIZONTAL_MIN_ANGLE_DEG;

    return manual_target;
}

/**
 * @brief 归位模式：平滑走回 0°；到位后启动波形。
 *        波形相位 0 对应 0°，所以归位终点和波形起点天然连续，不会跳。
 */
static fp32 homing_step(fp32 current_angle)
{
    fp32 step;

    if (homing_fresh)
    {
        homing_target = current_angle;
        homing_fresh = 0U;
    }

    step = RAIL_HOMING_SPEED * 0.001f;

    if (homing_target < -step)
    {
        homing_target += step;
    }
    else if (homing_target > step)
    {
        homing_target -= step;
    }
    else
    {
        homing_target = 0.0f;
        trajectory_set_horizontal_mode((pending_wave == RAIL_WAVE1) ? TRAJ_SINE
                                                                    : TRAJ_TRIANGLE);
        mode = pending_wave;
    }

    return homing_target;
}

fp32 rail_control_update(fp32 current_angle)
{
    fp32 out;

    /* 摩擦前馈默认不作用，只有手动档会按运动方向写非零值 */
    g_horizontal_ff = 0.0f;

    if (line_ready)
        handle_line();

    /* 链路超时 → SAFE */
    if (!frame_received ||
        (HAL_GetTick() - last_frame_tick) > RAIL_LINK_TIMEOUT_MS)
    {
        mode = RAIL_SAFE;
    }

    /* 全模式都开限位夹紧：手动档也夹在 ±1080°。
     * 想临时完全放开（推到机械端点才停），把这里改成 1U 即可。 */
    g_horizontal_limit_off = 0U;

    switch (mode)
    {
    case RAIL_MANUAL:
        out = manual_step(current_angle);
        break;

    case RAIL_HOMING:
        out = homing_step(current_angle);
        break;

    case RAIL_WAVE1:
    case RAIL_WAVE2:
        out = trajectory_get_horizontal();
        break;

    case RAIL_SAFE:
    default:
        out = current_angle;           /* 停住不动 */
        break;
    }

    g_dbg_rc_mode = (uint8_t)mode;
    g_dbg_rc_target = out;

    return out;
}
