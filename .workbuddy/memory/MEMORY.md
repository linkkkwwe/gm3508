# GUID_RAIL · 项目长期笔记

## 工程概况
- 单轴（水平轴）引导轨工程：STM32F407 + M3508 电机 + C620 电调，MDK-ARM 构建。
- 目录：`Application/`（业务代码）、`Core/`（CubeMX 生成）、`Drivers/`。
- 控制结构：**单环角度控制**。`motor_ctrl_update()` 里角度环 PID 输出直接当电流，
  `use_cascade` 恒为 0，无速度环、无 IMU。位置反馈来自 M3508 自带编码器（8192 线/转子圈），
  以**上电位置为 0° 参考**，减速比 19.2032。
- 控制周期 1ms：TIM6（PSC=83、ARR=999，定时器时钟 84MHz）置标志，主循环轮询。
  **注意主循环是轮询 `control_tick_flag`，不是中断里直接算。**
- SerialPlot 每 10ms 通过 USART1（115200）发 3 个整数：目标角×10、当前角×10、PID 输出。

## 关键约定
- 轨迹模式：`trajectory.c` 支持 `TRAJ_SINE` / `TRAJ_TRIANGLE` / `TRAJ_STOP`，
  上电默认由 `main.c` 的 `HORIZONTAL_START_MODE` 指定（当前为 `TRAJ_SINE`）。
- 角度限位（`Motor.h` 的 `HORIZONTAL_MIN/MAX_ANGLE_DEG`）靠 `constrain_float` 限制**目标角度**
  （不是实际角度）。2026-09-17 曾被临时放宽到 ±60° 以消除削顶，注意这是**放弃了软限位保护**，
  且那处注释没跟着改。机构真实行程未知，动它之前要问用户。
- PID 参数是全局变量（`g_horizontal_*`），设计为 Keil Debug 里实时改、下一帧生效；
  `motor_ctrl_update()` 每帧把增益同步进 PID 结构体，但**不清误差状态**。
  → 调参实验**不需要重新编译烧录**，在 Watch 窗口改数就行，这是最快的验证路径。

## 已知坑（截至 2026-09-17）
1. ~~`TRAJECTORY_AMPLITUDE = 60.0f` 与限位 ±30° 冲突 → 正弦被削顶成梯形波。~~
   用户已把限位放宽到 ±60° 解决，但**机构行程是否允许 ±60° 未确认**。
2. 三处注释与数值不一致：`TRAJECTORY_PERIOD_MS`(4000 vs 注释"6秒")、
   `g_horizontal_max_out`(16384 vs 注释 5000)、`g_horizontal_max_iout`(5000 vs 注释 0)、
   `HORIZONTAL_MIN/MAX_ANGLE_DEG`(±60 vs 注释"±30°限位")。
   改参数时容易只改数值漏改注释，排查时不要信注释。
3. 三通道的 Y 轴：2026-09-17 第三张放大图用"极值处置信差"判定为**共用一根轴**
   （实际角峰顶比目标角低 8px、谷底对称高 8px = 幅值衰减 1.5%，独立缩放不会有这个差）。
   但三路原始量级差 27 倍（角度 ±600 vs 电流 ±16384），画在一起仍看不出角度细节——
   调参时应拆开画，或把电流除以 16 再发。
4. `HAL_UART_Transmit` 阻塞约 1.3ms，每 10 个控制周期触发一次，会让个别周期变 2ms。
5. 反馈掉线判定 100ms，超时会 `motor_ctrl_clear` + `trajectory_init`，轨迹相位会重置。
6. **D 项缺 dt 归一化（最重要的坑）**：`pid.c` 用 `error[0]-error[1]` 当微分，没除以控制周期。
   1ms 周期下 Kd=200 的等效微分增益只有 0.2（电流单位/(°/s)），
   位置环阻尼比 ζ≈0.007、Q≈72，任何扰动都振铃约 4 秒不衰减（实测 5.5 Hz 等幅振荡）。
   要么把 Kd 放大到 ~2 万，要么改成 `Kd*(Δe/T)` 并把 Kd 设成十几，要么直接用 speed_rpm。
7. `Motor.c` 把 `speed_rpm` 用 `(void)speed_rpm;` 丢掉了——单环角度控制下这是唯一现成、
   且比裸差分干净的速度反馈源，加阻尼优先用它。
8. **`g_horizontal_ki = 0` 导致低频爬行（stick-slip 极限环）**：
   纯 PD 环在误差归零时输出也归零，没有积分项顶住静摩擦，电机被反复粘住。
   实测（放大截图）：每个轨迹周期里实际角会"完全静止"3~4 次，单次 60~350 ms，
   全程 19.6% 的时间完全没动；脱困瞬间以 2.7 倍目标速度猛冲。
   卡滞时长 ∝ 1/目标速度（极值处最长），且各周期在**逐像素相同的位置**重复
   → 确定性极限环，不是随机摩擦噪声。
   **调 Kp/Kd 治不了这个**，必须先给积分项 + 改善机械摩擦。
9. 注意区分两种"不平滑"：**高频抖动**（电流指令在振，查阻尼/增益）
   和**低频爬行**（位置本身一顿一顿，查积分项/摩擦）。看截图缩小图容易只看到前者，
   放大图才能看到后者。诊断时先问用户"是一顿一顿还是高频发麻"。

## Git / 仓库
- 远端：**`git@github.com:linkkkwwe/gm3508.git`**（Public）——https://github.com/linkkkwwe/gm3508
  分支 `main`，2026-09-17 首推，提交 `82ebb76`「3508未调参」，1480 文件 / 53.4 MB。
- **兄弟项目 `D:\mxproject\Guide_rail`**（远端 `guide_rail`）是同一硬件的另一份平行演进拷贝：
  那边用 vofa.c + simulation/*.m，这边用 trajectory.c + serialplot.c；两边 `pid.c` 完全相同。
  两边路径冲突（都占 `Core/`、`Drivers/`、`application/`），**不能合到同一条分支**。
- `.gitignore` 沿用 Guide_rail 那份（忽略 `*.o/*.d/*.crf/*.axf/*.hex/*.map`、`MDK-ARM/GUID_RAIL/`、
  `MDK-ARM/RTE/`、`DebugConfig/`），保留 `.uvprojx/.uvoptx/.scvd/startup_stm32f407xx.s`。
  重写前那份历史留在 `backup-before-clean` 分支 + `pre-clean-20260917` 标签（原提交 `df7c9c3`）。
- **本机没有 `gh` CLI 也没有 token** → 建远端仓库必须让用户自己在 github.com/new 建（空仓库）。
  SSH key `~/.ssh/id_ed25519`（注释 `guide_rail`）可直连 GitHub，账号 `linkkkwwe`。
- 已知怪学号：本机 `git push -u` / `git fetch` 报成功但 `refs/remotes/origin/main` 不落盘，
  要靠 `git rev-parse refs/remotes/origin/main` + `git ls-tree -r` 复核，别只信 push 的输出。

## 对象常数（2026-09-17 由实测波形反推，可直接复用）
- **Kt/J = 2.39 (°/s²) 每 1 个电流指令单位**（由 Kp=500 时 5.5 Hz 等幅振铃反推：
  Kt/J = (2πf)²/Kp）。有了它就能纯计算预测带宽/阻尼，不用反复试。
- **静摩擦等效 U_break ≈ 2450 指令单位（≈15% 量程）**：实测 Kp=500 时轴停住期间
  目标已走过 4.9° 才脱困，2450 ≈ 500×4.9。
- 由此得 Ki=0 时的死区：脱困误差 = 2450/Kp（Kp=500→4.9°、300→8.2°、800→3.1°），
  停滞时长 ≈ √(e·2/(A·ω²))**，与实测 310~350 ms 对得上**。
- **降 Kp 会加重拐角抖动**（唯一力矩来源是 Kp·e），这里常见的"降增益减抖"直觉是错的。
- 最对症的是 **摩擦/速度前馈**：轨迹解析，ω_ref/α_ref 闭式可得；
  前馈量取 ~0.5×U_break（按动摩擦量级，给满会因为静/动摩擦差反而推着走）。
- Kd 到位后（ζ≈0.6 需 Kd≈13500@Kp=500）脱困不再弹射，但裸差分会放大量化噪声，
  长期应改成 `Kd*(Δe/T)` 或改用 speed_rpm。
- Ki 在 `pid.c` 里没乘周期，可用量级仅个位数；Ki≥20 在模型里即发散/高频颤振。
