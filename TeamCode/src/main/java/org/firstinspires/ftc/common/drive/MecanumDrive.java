package org.firstinspires.ftc.common.drive;

/**
 * High-level mecanum drive wrapper:
 *  - Accepts chassis speeds in ROBOT frame (ChassisSpeeds, mm/s & rad/s).
 *  - Uses kinematics to compute wheel speeds.
 *  - Delegates to DriveIO for actual hardware commands (mm/s wheel targets).
 *
 * This is what TeleOp / Auto should talk to.
 */
public class MecanumDrive
{
    private final MecanumKinematics kinematics;
    private final DriveIO io;
    private final double maxWheelSpeedMmPerS;

    /**
     * @param kinematics shared mecanum kinematics (common)
     * @param io         team-specific DriveIO implementation
     * @param maxWheelSpeedMmPerS maximum allowed wheel speed command in mm/s (placeholder ok)
     */
    public MecanumDrive(MecanumKinematics kinematics, DriveIO io, double maxWheelSpeedMmPerS)
    {
        this.kinematics = kinematics;
        this.io = io;
        this.maxWheelSpeedMmPerS = maxWheelSpeedMmPerS;
    }

    /**
     * Drive with chassis speeds in ROBOT frame (mm/s, rad/s).
     *
     * <p>The resulting wheel speeds are normalized to {@code maxWheelSpeedMmPerS} before being
     * forwarded to the hardware layer.</p>
     *
     * <p>TeleOp / Auto should compute {@link ChassisSpeeds} in ROBOT frame (or convert from field
     * frame before calling this).</p>
     */
    public void driveRobotRelative(ChassisSpeeds speeds)
    {
        WheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        wheelSpeeds.normalize(Math.abs(maxWheelSpeedMmPerS));
        io.setWheelSpeeds(wheelSpeeds);
    }

    /**
     * Convenience: stop the robot.
     */
    public void stop()
    {
        io.setWheelSpeeds(new WheelSpeeds(0.0, 0.0, 0.0, 0.0));
    }

    /**
     * Expose measured wheel speeds in mm/s (if DriveIO supports it).
     */
    public WheelSpeeds getMeasuredWheelSpeeds()
    {
        return io.getWheelVelocities();
    }
}
