package org.firstinspires.ftc.common.geometry;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.common.drive.ChassisSpeeds;
import org.junit.Test;

public class FrameTransformTest
{
    private static final double EPS = 1e-6;

    @Test
    public void fieldToRobotAndBackAreInverse()
    {
        ChassisSpeeds fieldSpeeds = new ChassisSpeeds(500.0, -200.0, 1.2);
        double heading = Math.toRadians(37.0);

        ChassisSpeeds robotRelative = FrameTransform.fieldToRobot(fieldSpeeds, heading);
        ChassisSpeeds backToField = FrameTransform.robotToField(robotRelative, heading);

        assertEquals(fieldSpeeds.vx, backToField.vx, EPS);
        assertEquals(fieldSpeeds.vy, backToField.vy, EPS);
        assertEquals(fieldSpeeds.omega, backToField.omega, EPS);
    }

    @Test
    public void robotToFieldMatchesExpectedRotation()
    {
        double heading = Math.toRadians(90.0);
        ChassisSpeeds robotSpeeds = new ChassisSpeeds(100.0, 0.0, -0.5);
        ChassisSpeeds fieldSpeeds = FrameTransform.robotToField(robotSpeeds, heading);

        assertEquals(0.0, fieldSpeeds.vx, EPS);
        assertEquals(100.0, fieldSpeeds.vy, EPS);
        assertEquals(-0.5, fieldSpeeds.omega, EPS);
    }
}
