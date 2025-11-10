package org.firstinspires.ftc.team28770_SYSNG.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.common.control.HeadingController;
import org.firstinspires.ftc.common.control.SlewRateLimiter;
import org.firstinspires.ftc.common.control.TeleOpConfig;
import org.firstinspires.ftc.common.control.TeleOpDriveHelper;
import org.firstinspires.ftc.common.drive.ChassisSpeeds;
import org.firstinspires.ftc.common.drive.MecanumDrive;
import org.firstinspires.ftc.common.drive.MecanumKinematics;
import org.firstinspires.ftc.common.geometry.Pose2d;
import org.firstinspires.ftc.common.localization.Localizer;
import org.firstinspires.ftc.common.localization.PinpointLocalizer;
import org.firstinspires.ftc.team28770_SYSNG.Team28770Constants;
import org.firstinspires.ftc.team28770_SYSNG.drive.Team28770MecanumDriveIO;

@TeleOp(name = "Team28770 TeleOp", group = "28770")
public class Team28770TeleOp extends OpMode
{
    private MecanumDrive drive;
    private Localizer localizer;
    private TeleOpConfig teleOpConfig;
    private HeadingController headingController;
    private SlewRateLimiter vxLimiter;
    private SlewRateLimiter vyLimiter;
    private SlewRateLimiter omegaLimiter;

    private boolean headingHoldActive;
    private double headingHoldTarget;
    private double lastLoopTime;

    @Override
    public void init()
    {
        // Team-specific IO: real hardware here
        Team28770MecanumDriveIO io = new Team28770MecanumDriveIO(hardwareMap);
        // Common kinematics
        MecanumKinematics kinematics = new MecanumKinematics(
                Team28770Constants.WHEEL_BASE_MM,
                Team28770Constants.TRACK_WIDTH_MM
        );
        drive = new MecanumDrive(kinematics, io, Team28770Constants.MAX_WHEEL_SPEED_MM_PER_S);
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

        teleOpConfig = new TeleOpConfig(
                Team28770Constants.TELEOP_DEADBAND,
                Team28770Constants.TELEOP_EXPO_TRANSLATION,
                Team28770Constants.TELEOP_EXPO_ROTATION,
                Team28770Constants.TELEOP_MAX_VEL_MM_PER_S,
                Team28770Constants.TELEOP_MAX_ANG_VEL_RAD_PER_S
        );

        headingController = new HeadingController(
                Team28770Constants.HEADING_KP,
                Team28770Constants.HEADING_KI,
                Team28770Constants.HEADING_KD
        );

        vxLimiter = new SlewRateLimiter(Team28770Constants.TELEOP_LINEAR_SLEW_RATE);
        vyLimiter = new SlewRateLimiter(Team28770Constants.TELEOP_LINEAR_SLEW_RATE);
        omegaLimiter = new SlewRateLimiter(Team28770Constants.TELEOP_ANGULAR_SLEW_RATE);

        lastLoopTime = getRuntime();
    }

    @Override
    public void start()
    {
        Pose2d pose = localizer.getPose();
        headingController.reset(pose.heading);
        headingController.setTarget(pose.heading);
        headingHoldTarget = pose.heading;
        headingHoldActive = false;

        double now = getRuntime();
        vxLimiter.reset(0.0, now);
        vyLimiter.reset(0.0, now);
        omegaLimiter.reset(0.0, now);
        lastLoopTime = now;
    }

    @Override
    public void loop()
    {
        localizer.update();

        double now = getRuntime();
        double dt = Math.max(now - lastLoopTime, 1e-6);
        lastLoopTime = now;

        ChassisSpeeds command = TeleOpDriveHelper.robotRelativeFromJoysticks(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x,
                teleOpConfig
        );

        Pose2d pose = localizer.getPose();
        boolean rotationCommanded = Math.abs(gamepad1.right_stick_x) > teleOpConfig.deadband();
        double omegaCmd;
        if (rotationCommanded)
        {
            headingHoldActive = false;
            headingHoldTarget = pose.heading;
            headingController.setTarget(headingHoldTarget);
            omegaCmd = command.omega;
        }
        else
        {
            if (!headingHoldActive)
            {
                headingHoldTarget = pose.heading;
                headingController.reset(headingHoldTarget);
                headingController.setTarget(headingHoldTarget);
                headingHoldActive = true;
            }
            omegaCmd = headingController.update(pose.heading, dt);
        }

        double limitedVx = vxLimiter.calculate(command.vx, now);
        double limitedVy = vyLimiter.calculate(command.vy, now);
        double limitedOmega = omegaLimiter.calculate(omegaCmd, now);

        ChassisSpeeds limitedSpeeds = new ChassisSpeeds(limitedVx, limitedVy, limitedOmega);
        drive.driveRobotRelative(limitedSpeeds);

        telemetry.addData("cmd vx (mm/s)", limitedVx);
        telemetry.addData("cmd vy (mm/s)", limitedVy);
        telemetry.addData("cmd omega (rad/s)", limitedOmega);
        telemetry.addData("pose", localizer.getPose());
        telemetry.addData("heading hold", headingHoldActive);
        telemetry.addData("heading target", headingHoldTarget);
        telemetry.update();
    }
}
