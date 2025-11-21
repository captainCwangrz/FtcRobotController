package org.firstinspires.ftc.common.control;

/**
 * Configuration bundle for TeleOp input shaping.
 *
 * <p>All units obey the control stack ground rules:</p>
 * <ul>
 *     <li>{@code deadband}: unitless fraction of stick travel.</li>
 *     <li>{@code expoTranslation}: unitless, 0 = linear, 1 = strong curve.</li>
 *     <li>{@code expoRotation}: unitless, 0 = linear, 1 = strong curve.</li>
 *     <li>{@code maxVelMmPerS}: maximum linear velocity command, mm/s.</li>
 *     <li>{@code maxAngVelRadPerS}: maximum angular velocity command, rad/s.</li>
 * </ul>
 */
public final class TeleOpConfig {
    private final double deadband;
    private final double expoTranslation;
    private final double expoRotation;
    private final double maxVelMmPerS;
    private final double maxAngVelRadPerS;

    public TeleOpConfig(
            double deadband,
            double expoTranslation,
            double expoRotation,
            double maxVelMmPerS,
            double maxAngVelRadPerS
    ) {
        this.deadband = deadband;
        this.expoTranslation = expoTranslation;
        this.expoRotation = expoRotation;
        this.maxVelMmPerS = maxVelMmPerS;
        this.maxAngVelRadPerS = maxAngVelRadPerS;
    }

    public double deadband() {
        return deadband;
    }

    public double expoTranslation() {
        return expoTranslation;
    }

    public double expoRotation() {
        return expoRotation;
    }

    public double maxVelMmPerS() {
        return maxVelMmPerS;
    }

    public double maxAngVelRadPerS() {
        return maxAngVelRadPerS;
    }
}
