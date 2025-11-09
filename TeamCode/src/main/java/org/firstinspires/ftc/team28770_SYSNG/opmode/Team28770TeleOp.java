package org.firstinspires.ftc.team28770_SYSNG.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.common.drive.ChassisSpeeds;
import org.firstinspires.ftc.common.drive.MecanumDrive;
import org.firstinspires.ftc.common.drive.MecanumKinematics;
import org.firstinspires.ftc.common.localization.Localizer;
import org.firstinspires.ftc.common.localization.PinpointLocalizer;
import org.firstinspires.ftc.team28770_SYSNG.Team28770Constants;
import org.firstinspires.ftc.team28770_SYSNG.drive.Team28770MecanumDriveIO;

@TeleOp(name = "Team28770 TeleOp", group = "28770")
public class Team28770TeleOp extends OpMode
{
    private MecanumDrive drive;
    private Localizer localizer;

    // TeleOp max speeds (use real values after characterization)
    private static final double TELEOP_MAX_VEL = Team28770Constants.MAX_WHEEL_MPS * 0.8;
    private static final double TELEOP_MAX_ANG_VEL = Math.PI * 2.0; // rad/s

    @Override
    public void init()
    {
        // Team-specific IO: real hardware here
        Team28770MecanumDriveIO io = new Team28770MecanumDriveIO(hardwareMap);
        // Common kinematics
        MecanumKinematics kinematics = new MecanumKinematics(
                Team28770Constants.WHEEL_BASE,
                Team28770Constants.TRACK_WIDTH
        );
        drive = new MecanumDrive(kinematics, io);
        localizer = new PinpointLocalizer(hardwareMap, Team28770Constants.PINPOINT_NAME, Team28770Constants.PINPOINT_DIST_UNIT, Team28770Constants.PINPOINT_POD_TYPE, Team28770Constants.PINPOINT_X_OFFSET, Team28770Constants.PINPOINT_Y_OFFSET, Team28770Constants.PINPOINT_X_DIR, Team28770Constants.PINPOINT_Y_DIR, true);
    }

    @Override
    public void loop()
    {
        localizer.update();

        double lx = gamepad1.left_stick_x;
        double ly = gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        lx = applyDeadband(lx, 0.05);
        ly = applyDeadband(ly, 0.05);
        rx = applyDeadband(rx, 0.05);

        lx = cubic(lx);
        ly = cubic(ly);
        rx = cubic(rx);

        // Robot-centric mapping:
        double forward = -ly;
        double strafeRight = lx;
        double vx = forward * TELEOP_MAX_VEL;
        double vy = -strafeRight * TELEOP_MAX_VEL;
        double turn = -rx;
        double omega = turn * TELEOP_MAX_ANG_VEL;

        drive.driveRobotRelative(new ChassisSpeeds(vx, vy, omega));

        telemetry.addData("vx", vx);
        telemetry.addData("vy", vy);
        telemetry.addData("omega", omega);
        telemetry.addData("pose", localizer.getPose());
        telemetry.update();
    }

    private static double applyDeadband(double v, double t)
    {
        return Math.abs(v) > t ? v : 0.0;
    }

    private static double cubic(double x)
    {
        return x * x * x;
    }
}
