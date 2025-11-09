package org.firstinspires.ftc.common.localization;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.common.GoBildaPinpointDriver;
import org.firstinspires.ftc.common.drive.ChassisSpeeds;
import org.firstinspires.ftc.common.geometry.Pose2d;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

public class PinpointLocalizer implements Localizer
{
    private final GoBildaPinpointDriver pinpoint;
    private final DistanceUnit unit;
    private Pose2d pose = new Pose2d(0.0, 0.0, 0.0);
    private ChassisSpeeds velRobot = new ChassisSpeeds(0.0, 0.0, 0.0);
    private ChassisSpeeds accRobot = new ChassisSpeeds(0.0, 0.0, 0.0);
    private ChassisSpeeds lastVel = null;
    private long lastVelTimeNanos = 0L;

    /**
     * @param hw           HardwareMap
     * @param deviceName   Config name
     * @param unit         Unit for distances and velocities
     * @param podType      goBILDA pod type
     * @param xOffsetMm    X pod offset (mm)
     * @param yOffsetMm    Y pod offset (mm)
     * @param xDir         X pod direction
     * @param yDir         Y pod direction
     * @param resetOnInit  Whether to resetPosAndIMU automatically
     */
    public PinpointLocalizer(
            HardwareMap hw,
            String deviceName,
            DistanceUnit unit,
            GoBildaPinpointDriver.GoBildaOdometryPods podType,
            double xOffsetMm,
            double yOffsetMm,
            GoBildaPinpointDriver.EncoderDirection xDir,
            GoBildaPinpointDriver.EncoderDirection yDir,
            boolean resetOnInit
    )
    {
        this.unit = unit;
        this.pinpoint = hw.get(GoBildaPinpointDriver.class, deviceName);

        pinpoint.setEncoderResolution(podType);
        pinpoint.setOffsets(xOffsetMm, yOffsetMm, DistanceUnit.MM);
        pinpoint.setEncoderDirections(xDir, yDir);

        if (resetOnInit)
        {
            resetAll();
        }
    }

    @Override
    public void update()
    {
        pinpoint.update();

        // --- Pose ---
        Pose2D p = pinpoint.getPosition();
        double x = p.getX(unit);
        double y = p.getY(unit);
        double heading = p.getHeading(AngleUnit.RADIANS);
        pose = new Pose2d(x, y, heading);

        // --- Velocity  ---
        double vx = pinpoint.getVelX(unit);
        double vy = pinpoint.getVelY(unit);
        double omega = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);

        ChassisSpeeds newVel = new ChassisSpeeds(vx, vy, omega);
        velRobot = newVel;

        // --- Acceleration ---
        long now = System.nanoTime();
        if (lastVel != null && lastVelTimeNanos != 0L)
        {
            double dt = (now - lastVelTimeNanos) * 1e-9;
            if (dt > 1e-6) {
                double ax = (newVel.vx - lastVel.vx) / dt;
                double ay = (newVel.vy - lastVel.vy) / dt;
                double aOmega = (newVel.omega - lastVel.omega) / dt;
                accRobot = new ChassisSpeeds(ax, ay, aOmega);
            }
        }
        lastVel = newVel;
        lastVelTimeNanos = now;
    }

    @Override
    public Pose2d getPose()
    {
        return pose;
    }

    @Override
    public void setPose(Pose2d newPose)
    {
        if (newPose == null) return;

        Pose2D pinPose = new Pose2D(
                unit,
                newPose.x,
                newPose.y,
                AngleUnit.RADIANS,
                newPose.heading
        );
        pinpoint.setPosition(pinPose);

        pose = newPose;

        lastVel = null;
        lastVelTimeNanos = 0L;
        velRobot = new ChassisSpeeds(0.0, 0.0, 0.0);
        accRobot = new ChassisSpeeds(0.0, 0.0, 0.0);
    }

    public void resetAll()
    {
        pinpoint.resetPosAndIMU();
        pose = new Pose2d(0.0, 0.0, 0.0);
        lastVel = null;
        lastVelTimeNanos = 0L;
        velRobot = new ChassisSpeeds(0.0, 0.0, 0.0);
        accRobot = new ChassisSpeeds(0.0, 0.0, 0.0);
    }

    public ChassisSpeeds getRobotVelocity()
    {
        return velRobot;
    }

    public ChassisSpeeds getRobotAcceleration()
    {
        return accRobot;
    }
}
