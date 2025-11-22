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
     * Applies magnitude-based shaping to translation inputs and component-based shaping to rotation.
     *
     * @param lx  left stick x, unitless strafe command (+ left).
     * @param ly  left stick y, unitless forward command (+ up on stick).
     * @param rx  right stick x, unitless CCW rotation command.
     * @param cfg configuration describing deadband, expo, and limits.
     * @return shaped chassis speeds (mm/s, rad/s) without any frame rotation applied.
     */
    public static ChassisSpeeds shapeInputs(double lx, double ly, double rx, TeleOpConfig cfg)
    {
        // --- 1. TRANSLATION (Magnitude-Based with Noise Floor) ---

        // A. Apply a tiny noise floor to raw components to prevent "bleed-through" drift.
        //    This ensures that if you push straight forward, a  < 5% thumb slip doesn't cause a strafe.
        if (Math.abs(lx) < cfg.deadband()) lx = 0.0;
        if (Math.abs(ly) < cfg.deadband()) ly = 0.0;

        // B. Calculate the magnitude (length) of the joystick vector
        double rawMagnitude = Math.hypot(lx, ly);

        // C. Apply the main Deadband & Expo to the MAGNITUDE, not the axes.
        //    This ensures circular response (diagonal speed == forward speed).
        double shapedMagnitude = InputShaping.applyDeadband(rawMagnitude, cfg.deadband());
        shapedMagnitude = InputShaping.applyExpo(shapedMagnitude, cfg.expoTranslation());

        // D. Re-scale the original vector direction by the new shaped magnitude.
        double vx, vy;
        if (rawMagnitude > 1e-6)
        {
            double scaleFactor = shapedMagnitude / rawMagnitude;
            // Convert Joystick Frame (Up-Negative) to Robot Frame (Forward-Positive)
            vx = -ly * scaleFactor;
            vy = lx * scaleFactor;
        }
        else
        {
            vx = 0.0;
            vy = 0.0;
        }

        // E. Scale to physical limits (mm/s)
        //    Note: scale() clamps the output, but our math above guarantees [-1, 1] anyway.
        vx = InputShaping.scale(vx, cfg.maxVelMmPerS());
        vy = InputShaping.scale(vy, cfg.maxVelMmPerS());


        // --- 2. ROTATION (Component-Based) ---

        // Rotation is 1D, so we shape it directly. Magnitude logic doesn't apply here.
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
