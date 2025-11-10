package org.firstinspires.ftc.common.localization;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.common.GoBildaPinpointDriver;
import org.firstinspires.ftc.common.drive.ChassisSpeeds;
import org.firstinspires.ftc.common.geometry.MathUtil;
import org.firstinspires.ftc.common.geometry.Pose2d;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Objects;

/**
 * Localizer backed by the goBILDA Pinpoint driver returning mm & rad values.
 */
public class PinpointLocalizer implements Localizer
{
    private static final DistanceUnit OUTPUT_DISTANCE_UNIT = DistanceUnit.MM;
    private static final AngleUnit OUTPUT_ANGLE_UNIT = AngleUnit.RADIANS;
    private static final double MIN_DT_SECONDS = 1e-6;

    private static final ChassisSpeeds ZERO_SPEEDS = new ChassisSpeeds(0.0, 0.0, 0.0);

    private final GoBildaPinpointDriver pinpoint;
    private final DistanceUnit driverDistanceUnit;

    private Pose2d pose = zeroPose();
    private ChassisSpeeds velRobot = ZERO_SPEEDS;
    private ChassisSpeeds accRobot = ZERO_SPEEDS;
    private ChassisSpeeds lastVel = null;
    private long lastVelTimeNanos = 0L;

    /**
     * @param hw           HardwareMap
     * @param deviceName   Config name
     * @param distanceUnit Distance/velocity unit provided by the driver (use mm for common code)
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
            DistanceUnit distanceUnit,
            GoBildaPinpointDriver.GoBildaOdometryPods podType,
            double xOffsetMm,
            double yOffsetMm,
            GoBildaPinpointDriver.EncoderDirection xDir,
            GoBildaPinpointDriver.EncoderDirection yDir,
            boolean resetOnInit
    )
    {
        this.driverDistanceUnit = Objects.requireNonNull(distanceUnit, "distanceUnit");
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
        Pose2D rawPose = Objects.requireNonNull(pinpoint.getPosition(), "pinpoint position");
        double x = toOutputDistance(rawPose.getX(driverDistanceUnit));
        double y = toOutputDistance(rawPose.getY(driverDistanceUnit));
        double heading = MathUtil.wrapAngle(rawPose.getHeading(OUTPUT_ANGLE_UNIT));
        pose = new Pose2d(x, y, heading);

        // --- Velocity  ---
        double vx = toOutputDistance(pinpoint.getVelX(driverDistanceUnit));
        double vy = toOutputDistance(pinpoint.getVelY(driverDistanceUnit));
        double omega = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);

        ChassisSpeeds newVel = new ChassisSpeeds(vx, vy, omega);
        velRobot = newVel;

        // --- Acceleration ---
        long now = System.nanoTime();
        if (lastVel != null && lastVelTimeNanos != 0L)
        {
            double dt = (now - lastVelTimeNanos) * 1e-9;
            if (dt > MIN_DT_SECONDS)
            {
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
        Objects.requireNonNull(newPose, "pose");

        Pose2D pinPose = new Pose2D(
                driverDistanceUnit,
                driverDistanceUnit.fromUnit(OUTPUT_DISTANCE_UNIT, newPose.x),
                driverDistanceUnit.fromUnit(OUTPUT_DISTANCE_UNIT, newPose.y),
                OUTPUT_ANGLE_UNIT,
                newPose.heading
        );
        pinpoint.setPosition(pinPose);

        pose = new Pose2d(newPose.x, newPose.y, newPose.heading);

        lastVel = null;
        lastVelTimeNanos = 0L;
        velRobot = ZERO_SPEEDS;
        accRobot = ZERO_SPEEDS;
    }

    public void resetAll()
    {
        pinpoint.resetPosAndIMU();
        pose = zeroPose();
        lastVel = null;
        lastVelTimeNanos = 0L;
        velRobot = ZERO_SPEEDS;
        accRobot = ZERO_SPEEDS;
    }

    public ChassisSpeeds getRobotVelocity()
    {
        return velRobot;
    }

    public ChassisSpeeds getRobotAcceleration()
    {
        return accRobot;
    }

    private static Pose2d zeroPose()
    {
        return new Pose2d(0.0, 0.0, 0.0);
    }

    private double toOutputDistance(double value)
    {
        return OUTPUT_DISTANCE_UNIT.fromUnit(driverDistanceUnit, value);
    }
}
