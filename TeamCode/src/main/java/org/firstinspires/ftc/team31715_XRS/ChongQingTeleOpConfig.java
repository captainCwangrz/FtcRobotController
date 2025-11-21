package org.firstinspires.ftc.team31715_XRS;

/**
 * ChongQingTeleOp 手动程序配置
 *
 * 所有“手感相关”和“电机/编码器相关”的参数
 *
 */
public final class ChongQingTeleOpConfig {

    private ChongQingTeleOpConfig() {}

    // ================== 手感相关：整体速度 & 摇杆处理 ==================

    // 【全局】速度上限比例（0~1）
    // 调小：不容易失控
    // 调大：速度更快
    public static final double DRIVE_LIMIT = 0.9;

    // 摇杆死区
    // 小范围抖动忽略，避免轻微误触
    public static final double DEADBAND = 0.04;

    // 是否对摇杆输入做平方曲线处理
    // 低速稳定，最高速不影响
    public static final boolean SQUARE_INPUT = true;

    // 旋转对前进的混合比例：转向时两侧前进量差值
    // 数值越大，原地转向更干脆
    // 太大会影响直行手感
    // Todo: 道理上应该是物理标定出来一个数字，v2再说
    public static final double ROT_MIX = 0.30;


    // ================== 平移加减速限制 ==================

    // 左右两侧驱动量变化速度
    public static final double SLEW_ACCEL = 2.0; // 加速
    public static final double SLEW_DECEL = 24.0; // 减速
    public static final double SLEW_DECEL_FLIP = 48.0; // 反向180时的减速


    // ================== 旋转加减速限制 ==================

    public static final double SLEW_R_ACCEL = 5.0;
    public static final double SLEW_R_DECEL = 40.0;
    public static final double SLEW_R_DECEL_FLIP = 80.0;


    // ================== 模块转向细节 ==================

    // 避免模组进行”翻转选择”的时候来回抖
    // 一般不需要调整
    public static final double CHOICE_HYST_DEG = 8.0;

    // 无输入情况下模组朝向保持阈值
    // 一般不需要调整
    public static final double HOLD_MAG_THRESH = 0.02;

    // 余弦抑制：当轮子角度偏差大时自动减小驱动，减少打滑
    public static final boolean COS_SUPPRESS = true; // 开关
    public static final double COS_POWER = 1.0; // 1.0 ~ 2.0


    // ================== 自动保角度（Heading Hold） ==================

    // 右摇杆 X 超过该值视为“驾驶员主动旋转”，此时不保角度
    public static final double ROTATE_INTENT_THRESH = 0.04;

    // 是否反转误差方向（根据你定义的顺逆时针方向来定）
    // 不要轻易调整
    public static final boolean FLIP_HEADING_ERR = true;

    // P、D 参数
    public static final double HEADING_KP = 0.70;
    public static final double HEADING_KD = 0.14;

    // 角度误差死区（弧度）
    public static final double HEADING_DEAD_RAD = Math.toRadians(0.5);

    // 惯力消散阈值
    public static final double HEADING_VEL_DEAD = Math.toRadians(4.0);

    // 当角速度下降到这个值以下时，记录当前角度为新的保持目标
    public static final double HOLD_ENGAGE_VEL = Math.toRadians(8.0);

    // 保角度时允许叠加的最大旋转修正指令，防止过猛
    public static final double HEADING_MAX_CORR = 0.80;


    // ================== 驱动与闭环参数 ==================
    // ToDo: 每个大迭代再重新标定

    public static final double MAX_MOTOR_TPS = 2700.0;
    public static final double MAX_DIFF_TPS = 3780.0;
    public static final double STEER_KP = 1500.0;
    public static final double MAX_STEER_TPS = 2300.0;
    public static final double DRIVE_KP = 0.10;


    // ================== 机械参数 ==================
    // !!!! 未发生机械结构变化或未经严格多方审阅，禁止更改 !!!!

    public static final double RAD_PER_SUMTICK_L = (2.0 * Math.PI) / 1866.0;
    public static final double RAD_PER_SUMTICK_R = (2.0 * Math.PI) / 1865.0;
    public static final double PINPOINT_X_OFFSET_MM = -69.0;
    public static final double PINPOINT_Y_OFFSET_MM = -130.0;
}
