package org.firstinspires.ftc.team28770_SYSNG.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.common.control.HeadingController;
import org.firstinspires.ftc.common.control.SlewRateLimiter;
import org.firstinspires.ftc.common.control.TeleOpConfig;
import org.firstinspires.ftc.common.control.TeleOpDriveHelper;
import org.firstinspires.ftc.common.drive.ChassisSpeeds;
import org.firstinspires.ftc.common.drive.MecanumDrive;
import org.firstinspires.ftc.common.drive.MecanumKinematics;
import org.firstinspires.ftc.common.geometry.FrameTransform;
import org.firstinspires.ftc.common.geometry.Pose2d;
import org.firstinspires.ftc.common.geometry.Vector2d;
import org.firstinspires.ftc.common.localization.Localizer;
import org.firstinspires.ftc.common.localization.PinpointLocalizer;
import org.firstinspires.ftc.team28770_SYSNG.Team28770Constants;
import org.firstinspires.ftc.team28770_SYSNG.drive.Team28770MecanumDriveIO;

@TeleOp(name = "Team28770 TeleOp", group = "28770")
public class Team28770TeleOp extends OpMode
{
    // Subsystems
    private MecanumDrive drive;
    private Localizer localizer;

    // Controllers
    private TeleOpConfig teleOpConfig;
    private HeadingController headingController;

    // Slew Limiters (manual only)
    private SlewRateLimiter vxLimiter;
    private SlewRateLimiter vyLimiter;
    private SlewRateLimiter omegaLimiter;

    // States
    private boolean headingHoldActive = false;
    private double headingHoldTarget = 0.0;
    private boolean isFieldCentric = false;
    private double lastLoopTime = 0.0;
    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void init()
    {
        // io init
        Team28770MecanumDriveIO io = new Team28770MecanumDriveIO(hardwareMap);

        // kinematics init
        MecanumKinematics kinematics = new MecanumKinematics(Team28770Constants.WHEEL_BASE_MM, Team28770Constants.TRACK_WIDTH_MM);

        // drive init
        drive = new MecanumDrive(kinematics, io, Team28770Constants.MAX_WHEEL_SPEED_MM_PER_S);

        // pinpoint localizer init
        localizer = new PinpointLocalizer(
                hardwareMap,
                Team28770Constants.PINPOINT_NAME,
                Team28770Constants.PINPOINT_DIST_UNIT,
                Team28770Constants.PINPOINT_POD_TYPE,
                Team28770Constants.PINPOINT_X_OFFSET,
                Team28770Constants.PINPOINT_Y_OFFSET,
                Team28770Constants.PINPOINT_X_DIR,
                Team28770Constants.PINPOINT_Y_DIR,
                true
        );

        // input shaping configs
        teleOpConfig = new TeleOpConfig(
                Team28770Constants.TELEOP_DEADBAND,
                Team28770Constants.TELEOP_EXPO_TRANSLATION,
                Team28770Constants.TELEOP_EXPO_ROTATION,
                Team28770Constants.TELEOP_MAX_VEL_MM_PER_S,
                Team28770Constants.TELEOP_MAX_ANG_VEL_RAD_PER_S
        );

        // heading controller init
        headingController = new HeadingController(
                Team28770Constants.HEADING_KP,
                Team28770Constants.HEADING_KI,
                Team28770Constants.HEADING_KD,
                Team28770Constants.HEADING_KS,
                Team28770Constants.HEADING_ERR_DEADBAND_RAD
        );

        // slew rate limiter init
        vxLimiter = new SlewRateLimiter(
                Team28770Constants.TELEOP_LINEAR_ACCEL_SLEW,
                Team28770Constants.TELEOP_LINEAR_DECEL_SLEW
        );
        vyLimiter = new SlewRateLimiter(
                Team28770Constants.TELEOP_LINEAR_ACCEL_SLEW,
                Team28770Constants.TELEOP_LINEAR_DECEL_SLEW
        );
        omegaLimiter = new SlewRateLimiter(
                Team28770Constants.TELEOP_ANG_ACCEL_SLEW,
                Team28770Constants.TELEOP_ANG_DECEL_SLEW
        );

        telemetry.addLine("Init complete");
        telemetry.update();
    }

    @Override
    public void start()
    {
        timer.reset();
        lastLoopTime = timer.seconds();

        localizer.update();
        Pose2d startPose = localizer.getPose();
        ChassisSpeeds startVel = localizer.getRobotVelocity();
        headingHoldTarget = startPose.heading;
        headingHoldActive = true;

        headingController.reset(startPose.heading);
        headingController.setTarget(headingHoldTarget);

        vxLimiter.reset(startVel.vx, lastLoopTime);
        vyLimiter.reset(startVel.vy, lastLoopTime);
        omegaLimiter.reset(startVel.omega, lastLoopTime);
    }

    @Override
    public void loop()
    {
        // update states
        localizer.update();
        Pose2d pose = localizer.getPose();

        double now = timer.seconds();
        double dt = now - lastLoopTime;
        lastLoopTime = now;

        if (gamepad1.leftBumperWasPressed())
        {
            isFieldCentric = !isFieldCentric;
            vxLimiter.reset(0.0, now);
            vyLimiter.reset(0.0, now);
            omegaLimiter.reset(0.0, now);
        }

        // joystick inputs
        double lx = -gamepad1.left_stick_x;
        double ly = gamepad1.left_stick_y;
        double rx = -gamepad1.right_stick_x;

        // Shape rotation (robot frame)
        double omegaCmd = TeleOpDriveHelper.shapeInputs(0.0, 0.0, rx, teleOpConfig).omega;

        // heading lock
        boolean rotationIntent = Math.abs(omegaCmd) > 1e-4;
        double finalOmegaCmd;
        if (rotationIntent)
        {
            headingHoldActive = false;
            finalOmegaCmd = omegaLimiter.calculate(omegaCmd, now);
            headingHoldTarget = pose.heading;
        }
        else
        {
            if (!headingHoldActive)
            {
                headingHoldTarget = pose.heading;
                headingController.reset(pose.heading);
                headingController.setTarget(headingHoldTarget);
                headingHoldActive = true;
            }
            omegaLimiter.reset(0.0, now);
            finalOmegaCmd = headingController.update(pose.heading, dt);
        }

        double limitedVx;
        double limitedVy;

        if (isFieldCentric)
        {
            ChassisSpeeds fieldCmd = TeleOpDriveHelper.shapeInputs(lx, ly, 0.0, teleOpConfig);
            double vxFieldLimited = vxLimiter.calculate(fieldCmd.vx, now);
            double vyFieldLimited = vyLimiter.calculate(fieldCmd.vy, now);
            ChassisSpeeds robotVel = FrameTransform.fieldToRobot(
                    new ChassisSpeeds(vxFieldLimited, vyFieldLimited, 0.0),
                    pose.heading);
            limitedVx = robotVel.vx;
            limitedVy = robotVel.vy;
        }
        else
        {
            ChassisSpeeds robotRaw = TeleOpDriveHelper.shapeInputs(lx, ly, 0.0, teleOpConfig);
            limitedVx = vxLimiter.calculate(robotRaw.vx, now);
            limitedVy = vyLimiter.calculate(robotRaw.vy, now);
        }

        // send commands to drivetrain
        ChassisSpeeds finalSpeeds = new ChassisSpeeds(limitedVx, limitedVy, finalOmegaCmd);
        drive.driveRobotRelative(finalSpeeds);

        // telemetry
        telemetry.addData("Mode", headingHoldActive ? "LOCKED" : "MANUAL");
        telemetry.addData("Heading Err (deg)", Math.toDegrees(headingHoldTarget - pose.heading));
        telemetry.update();
    }
}
