package org.firstinspires.ftc.common.geometry;

import org.firstinspires.ftc.common.drive.ChassisSpeeds;

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

    /**
     * Convert chassis speeds expressed in the field frame into the robot frame.
     *
     * @param fieldSpeeds chassis speeds where vx, vy are in the field frame, mm/s.
     * @param heading     robot heading in the field frame, radians.
     * @return equivalent robot-relative chassis speeds.
     */
    public static ChassisSpeeds fieldToRobot(ChassisSpeeds fieldSpeeds, double heading)
    {
        Vector2d robotVel = fieldToRobotVel(fieldSpeeds.vx, fieldSpeeds.vy, heading);
        return new ChassisSpeeds(robotVel.x, robotVel.y, fieldSpeeds.omega);
    }

    /**
     * Convert chassis speeds expressed in the robot frame into the field frame.
     *
     * @param robotSpeeds chassis speeds where vx, vy are in the robot frame, mm/s.
     * @param heading     robot heading in the field frame, radians.
     * @return equivalent field-relative chassis speeds.
     */
    public static ChassisSpeeds robotToField(ChassisSpeeds robotSpeeds, double heading)
    {
        Vector2d fieldVel = robotToFieldVel(robotSpeeds.vx, robotSpeeds.vy, heading);
        return new ChassisSpeeds(fieldVel.x, fieldVel.y, robotSpeeds.omega);
    }
}
