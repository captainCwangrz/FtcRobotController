package org.firstinspires.ftc.common.geometry;

/**
 * Transforms between Field frame (F) and Robot frame (R).
 *
 * Heading is the robot's orientation in the field frame:
 *  - radians, CCW+, 0 aligned with field +x.
 */
public final class FrameTransform
{
    private FrameTransform() {}

    /**
     * Transform velocity from field frame to robot frame.
     *
     * v_R = R(-θ) * v_F
     */
    public static Vector2d fieldToRobotVel(double vxField, double vyField, double heading)
    {
        double c = Math.cos(heading);
        double s = Math.sin(heading);
        double vxR =  c * vxField + s * vyField;
        double vyR = -s * vxField + c * vyField;
        return new Vector2d(vxR, vyR);
    }

    /**
     * Transform velocity from robot frame to field frame.
     *
     * v_F = R(θ) * v_R
     */
    public static Vector2d robotToFieldVel(double vxRobot, double vyRobot, double heading)
    {
        double c = Math.cos(heading);
        double s = Math.sin(heading);
        double vxF = c * vxRobot - s * vyRobot;
        double vyF = s * vxRobot + c * vyRobot;
        return new Vector2d(vxF, vyF);
    }
}
