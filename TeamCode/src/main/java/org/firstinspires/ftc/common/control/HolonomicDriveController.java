package org.firstinspires.ftc.common.control;

import org.firstinspires.ftc.common.drive.ChassisSpeeds;
import org.firstinspires.ftc.common.geometry.FrameTransform;
import org.firstinspires.ftc.common.geometry.Pose2d;

/**
 * Field-centric holonomic drive controller with simple PID + feedforward regulation.
 */
public class HolonomicDriveController
{
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController thetaController;

    public HolonomicDriveController(PIDController xController, PIDController yController, PIDController thetaController)
    {
        this.xController = xController;
        this.yController = yController;
        this.thetaController = thetaController;
        this.thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public ChassisSpeeds calculate(
            Pose2d current,
            Pose2d referencePose,
            ChassisSpeeds referenceVel,
            double dt)
    {
        xController.setSetpoint(referencePose.x);
        yController.setSetpoint(referencePose.y);
        thetaController.setSetpoint(referencePose.heading);

        double vxField = xController.calculate(current.x, dt) + referenceVel.vx;
        double vyField = yController.calculate(current.y, dt) + referenceVel.vy;
        double omega = thetaController.calculate(current.heading, dt) + referenceVel.omega;

        ChassisSpeeds fieldSpeeds = new ChassisSpeeds(vxField, vyField, omega);
        return FrameTransform.fieldToRobot(fieldSpeeds, current.heading);
    }
}
