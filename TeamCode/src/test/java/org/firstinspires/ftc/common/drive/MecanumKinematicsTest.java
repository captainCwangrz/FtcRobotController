package org.firstinspires.ftc.common.drive;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class MecanumKinematicsTest
{
    private static final double EPS = 1e-6;

    @Test
    public void forwardAndInverseAreConsistent()
    {
        MecanumKinematics kinematics = new MecanumKinematics(300.0, 260.0);
        WheelSpeeds wheelSpeeds = new WheelSpeeds(420.0, -180.0, 210.0, 75.0);

        ChassisSpeeds chassis = kinematics.toChassisSpeeds(wheelSpeeds);
        WheelSpeeds recovered = kinematics.toWheelSpeeds(chassis);

        assertEquals(wheelSpeeds.fl, recovered.fl, EPS);
        assertEquals(wheelSpeeds.fr, recovered.fr, EPS);
        assertEquals(wheelSpeeds.bl, recovered.bl, EPS);
        assertEquals(wheelSpeeds.br, recovered.br, EPS);
    }

    @Test
    public void pureRotationProducesOpposingWheelSpeeds()
    {
        MecanumKinematics kinematics = new MecanumKinematics(320.0, 320.0);
        ChassisSpeeds rotation = new ChassisSpeeds(0.0, 0.0, 1.0);

        WheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(rotation);

        assertEquals(-wheelSpeeds.bl, wheelSpeeds.fl, EPS);
        assertEquals(-wheelSpeeds.br, wheelSpeeds.fr, EPS);
    }
}
