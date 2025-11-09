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

    /**
     * @param kinematics shared mecanum kinematics (common)
     * @param io         team-specific DriveIO implementation
     */
    public MecanumDrive(MecanumKinematics kinematics, DriveIO io)
    {
        this.kinematics = kinematics;
        this.io = io;
    }

    /**
     * Drive with chassis speeds in ROBOT frame (mm/s, rad/s).
     *
     * TeleOp / Auto should compute ChassisSpeeds in ROBOT frame
     * (or convert from field frame before calling this).
     */
    public void driveRobotRelative(ChassisSpeeds speeds)
    {
        WheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        io.setWheelSpeeds(wheelSpeeds);
    }

    /**
     * Convenience: stop the robot.
     */
    public void stop()
    {
        io.setWheelSpeeds(new WheelSpeeds(0, 0, 0, 0));
    }

    /**
     * Expose measured wheel speeds in mm/s (if DriveIO supports it).
     */
    public WheelSpeeds getMeasuredWheelSpeeds()
    {
        return io.getWheelVelocities();
    }
}
