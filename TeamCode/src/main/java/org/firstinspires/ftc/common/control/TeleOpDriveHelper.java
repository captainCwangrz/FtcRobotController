package org.firstinspires.ftc.common.control;

import org.firstinspires.ftc.common.drive.ChassisSpeeds;
import org.firstinspires.ftc.common.geometry.FrameTransform;

/**
 * Helpers for mapping joystick inputs into chassis velocity commands.
 */
public final class TeleOpDriveHelper
{
    private TeleOpDriveHelper() {}

    /**
     * Applies deadband, exponential scaling, and velocity limits to raw joystick inputs.
     *
     * <p><b>Coordinate Frame Note:</b></p>
     * <ul>
     * <li>{@code vx, vy}: Returned in the <b>Input Frame</b> (Joystick Up/Left).
     * If used for Field-Centric drive, these represent Field North (+x) / West (+y).
     * If used for Robot-Centric drive, these represent Robot Forward (+x) / Left (+y).</li>
     * <li>{@code omega}: Always returned in the <b>Robot Frame</b> (CCW rotation rate).
     * This value is invariant under frame rotation.</li>
     * </ul>
     *
     * @param lx  left stick x, unitless strafe command (+ left).
     * @param ly  left stick y, unitless forward command (+ up on stick).
     * @param rx  right stick x, unitless CCW rotation command.
     * @param cfg configuration describing deadband, expo, and limits.
     * @return shaped chassis speeds (mm/s, rad/s) without any frame rotation applied.
     */
    public static ChassisSpeeds shapeInputs(double lx, double ly, double rx, TeleOpConfig cfg)
    {
        double vxInput = InputShaping.applyDeadband(-ly, cfg.deadband());
        vxInput = InputShaping.applyExpo(vxInput, cfg.expoTranslation());
        double vx = InputShaping.scale(vxInput, cfg.maxVelMmPerS());

        double vyInput = InputShaping.applyDeadband(lx, cfg.deadband());
        vyInput = InputShaping.applyExpo(vyInput, cfg.expoTranslation());
        double vy = InputShaping.scale(vyInput, cfg.maxVelMmPerS());

        double omegaInput = InputShaping.applyDeadband(rx, cfg.deadband());
        omegaInput = InputShaping.applyExpo(omegaInput, cfg.expoRotation());
        double omega = InputShaping.scale(omegaInput, cfg.maxAngVelRadPerS());

        return new ChassisSpeeds(vx, vy, omega);
    }

    /**
     * Convert raw joystick values into robot-relative chassis speeds.
     *
     * @param lx  left stick x, unitless strafe command (+ left).
     * @param ly  left stick y, unitless forward command (+ up on stick).
     * @param rx  right stick x, unitless CCW rotation command.
     * @param cfg configuration describing deadband, expo, and limits.
     * @return robot-frame chassis speeds in mm/s and rad/s.
     */
    public static ChassisSpeeds robotRelativeFromJoysticks(double lx, double ly, double rx, TeleOpConfig cfg)
    {
        // In robot-centric mode, the input frame already matches the robot frame.
        return shapeInputs(lx, ly, rx, cfg);
    }

    /**
     * Convert raw joystick values into field-relative chassis speeds and return the equivalent robot-frame command.
     *
     * @param lx       left stick x, unitless strafe command (+ field left).
     * @param ly       left stick y, unitless forward command (+ field forward).
     * @param rx       right stick x, unitless CCW rotation command.
     * @param heading  current robot heading in field frame, radians.
     * @param cfg      configuration describing deadband, expo, and limits.
     * @return robot-frame chassis speeds corresponding to the requested field-frame translation.
     */
    public static ChassisSpeeds fieldRelativeFromJoysticks(double lx, double ly, double rx, double heading, TeleOpConfig cfg)
    {
        ChassisSpeeds shaped = shapeInputs(lx, ly, rx, cfg);
        ChassisSpeeds fieldSpeeds = new ChassisSpeeds(shaped.vx, shaped.vy, shaped.omega);
        return FrameTransform.fieldToRobot(fieldSpeeds, heading);
    }
}
