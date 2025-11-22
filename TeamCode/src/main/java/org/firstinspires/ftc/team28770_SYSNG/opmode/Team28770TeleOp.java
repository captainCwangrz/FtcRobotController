package org.firstinspires.ftc.team28770_SYSNG.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.common.control.TeleOpConfig;
import org.firstinspires.ftc.common.control.TeleOpDriveHelper;
import org.firstinspires.ftc.common.drive.ChassisSpeeds;
import org.firstinspires.ftc.common.drive.MecanumDrive;
import org.firstinspires.ftc.common.drive.MecanumKinematics;
import org.firstinspires.ftc.team28770_SYSNG.Team28770Constants;
import org.firstinspires.ftc.team28770_SYSNG.drive.Team28770MecanumDriveIO;

@TeleOp(name = "Team28770TeleOp", group = "28770")
public class Team28770TeleOp extends OpMode
{
    // Subsystems
    private MecanumDrive drive;

    // Configuration
    private TeleOpConfig teleOpConfig;

    @Override
    public void init()
    {
        // 1. Initialize Hardware IO (Motors)
        Team28770MecanumDriveIO io = new Team28770MecanumDriveIO(hardwareMap);

        // 2. Initialize Kinematics (Math)
        MecanumKinematics kinematics = new MecanumKinematics(
                Team28770Constants.WHEEL_BASE_MM,
                Team28770Constants.TRACK_WIDTH_MM
        );

        // 3. Initialize Drive Subsystem
        drive = new MecanumDrive(
                kinematics,
                io,
                Team28770Constants.MAX_WHEEL_SPEED_MM_PER_S
        );

        // 4. Setup Input Shaping (Deadband, Expo curves, Speed Limits)
        teleOpConfig = new TeleOpConfig(
                Team28770Constants.TELEOP_DEADBAND,
                Team28770Constants.TELEOP_EXPO_TRANSLATION,
                Team28770Constants.TELEOP_EXPO_ROTATION,
                Team28770Constants.TELEOP_MAX_VEL_MM_PER_S,
                Team28770Constants.TELEOP_MAX_ANG_VEL_RAD_PER_S
        );

        telemetry.addLine("Basic TeleOp Initialized");
        telemetry.update();
    }

    @Override
    public void loop()
    {
        // 1. Read Joystick Inputs (Robot-Centric)
        //    Note: Gamepad Y is negative up, so we invert it.
        double lx = -gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y; // Forward is negative on stick, positive in math
        double rx = -gamepad1.right_stick_x;

        // 2. Shape Inputs
        //    Applies deadband, exponential curve, and scales to mm/s and rad/s
        ChassisSpeeds targetSpeeds = TeleOpDriveHelper.shapeInputs(lx, ly, rx, teleOpConfig);

        // 3. Send to Drive
        //    Converts chassis speeds (Robot Frame) -> Wheel speeds -> Motor ticks
        drive.driveRobotRelative(targetSpeeds);

        // 4. Telemetry
        telemetry.addData("Target vx", "%.1f mm/s", targetSpeeds.vx);
        telemetry.addData("Target vy", "%.1f mm/s", targetSpeeds.vy);
        telemetry.addData("Target w", "%.2f rad/s", targetSpeeds.omega);
        telemetry.update();
    }
}