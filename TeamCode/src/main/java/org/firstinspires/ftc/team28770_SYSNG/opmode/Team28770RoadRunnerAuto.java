package org.firstinspires.ftc.team28770_SYSNG.opmode;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

import org.firstinspires.ftc.common.drive.ChassisSpeeds;
import org.firstinspires.ftc.common.drive.MecanumDrive;
import org.firstinspires.ftc.common.localization.PinpointLocalizer;
import org.firstinspires.ftc.common.rr.RRAdapterUtil;
import org.firstinspires.ftc.common.rr.RRHolonomicDriveAdapter;
import org.firstinspires.ftc.team28770_SYSNG.Team28770Constants;
import org.firstinspires.ftc.team28770_SYSNG.drive.Team28770MecanumDriveIO;

/**
 * Example Autonomous demonstrating RoadRunner integration through the common drive stack.
 */
@Autonomous(name = "Team28770 RR Auto", group = "28770")
public class Team28770RoadRunnerAuto extends LinearOpMode
{
    private static final double MILLIMETERS_PER_METER = 1000.0;
    private static final TrajectoryBuilderParams TRAJECTORY_PARAMS = new TrajectoryBuilderParams(
            1e-6,
            new ProfileParams(0.25, 0.1, 1e-4)
    );

    // TODO: tune follower gains for your robot
    private static final double FOLLOWER_AXIAL_KP = 6.0;
    private static final double FOLLOWER_LATERAL_KP = 6.0;
    private static final double FOLLOWER_HEADING_KP = 4.0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        Team28770MecanumDriveIO io = new Team28770MecanumDriveIO(hardwareMap);
        org.firstinspires.ftc.common.drive.MecanumKinematics kinematics = new org.firstinspires.ftc.common.drive.MecanumKinematics(
                Team28770Constants.WHEEL_BASE_MM,
                Team28770Constants.TRACK_WIDTH_MM
        );
        MecanumDrive drive = new MecanumDrive(kinematics, io, Team28770Constants.MAX_WHEEL_SPEED_MM_PER_S);
        PinpointLocalizer localizer = new PinpointLocalizer(
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
        RRHolonomicDriveAdapter rrDrive = new RRHolonomicDriveAdapter(drive, localizer);

        org.firstinspires.ftc.common.geometry.Pose2d startPose = new org.firstinspires.ftc.common.geometry.Pose2d(0.0, 0.0, 0.0);
        localizer.setPose(startPose);

        double trackWidthMeters = Team28770Constants.TRACK_WIDTH_MM / MILLIMETERS_PER_METER;
        double wheelBaseMeters = Team28770Constants.WHEEL_BASE_MM / MILLIMETERS_PER_METER;
        double maxWheelVel = Team28770Constants.MAX_WHEEL_SPEED_MM_PER_S / MILLIMETERS_PER_METER;
        double maxAccelMps2 = Team28770Constants.AUTO_MAX_ACCEL_MM_PER_S2 / MILLIMETERS_PER_METER;
        double maxAngVel = Team28770Constants.AUTO_MAX_ANG_VEL_RAD_PER_S;
        MecanumKinematics rrKinematics = new MecanumKinematics(
                trackWidthMeters,
                wheelBaseMeters
        );

        MinVelConstraint velConstraint = new MinVelConstraint(Arrays.asList(
                new AngularVelConstraint(maxAngVel),
                rrKinematics.new WheelVelConstraint(maxWheelVel)
        ));
        ProfileAccelConstraint accelConstraint = new ProfileAccelConstraint(
                -maxAccelMps2,
                maxAccelMps2
        );

        Trajectory trajectory = new TrajectoryBuilder(
                TRAJECTORY_PARAMS,
                RRAdapterUtil.toRoadRunnerPose(startPose),
                0.0,
                velConstraint,
                accelConstraint
        )
                .splineTo(new Vector2d(0.6, 0.0), 0.0)
                .splineTo(new Vector2d(0.6, 0.6), Math.PI / 2.0)
                .build();

        TimeTrajectory timeTrajectory = new TimeTrajectory(trajectory);
        HolonomicController follower = new HolonomicController(
                FOLLOWER_AXIAL_KP,
                FOLLOWER_LATERAL_KP,
                FOLLOWER_HEADING_KP
        );

        ElapsedTime timer = new ElapsedTime();

        telemetry.addLine("RR Auto ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested())
        {
            return;
        }

        timer.reset();

        while (opModeIsActive())
        {
            localizer.update();

            double elapsed = timer.seconds();
            if (elapsed > timeTrajectory.duration)
            {
                break;
            }

            Pose2dDual<Time> targetState = timeTrajectory.get(elapsed);
            Pose2d targetPose = targetState.value();

            org.firstinspires.ftc.common.geometry.Pose2d fieldPose = localizer.getPose();
            Pose2d currentPose = RRAdapterUtil.toRoadRunnerPose(fieldPose);

            ChassisSpeeds robotVel = localizer.getRobotVelocity();
            PoseVelocity2d actualVel = toRoadRunnerVelocity(robotVel);

            PoseVelocity2dDual<Time> commandDual = follower.compute(
                    targetState,
                    currentPose,
                    actualVel
            );

            PoseVelocity2d command = commandDual.value();
            ChassisSpeeds robotCmd = new ChassisSpeeds(
                    command.linearVel.x * MILLIMETERS_PER_METER,
                    command.linearVel.y * MILLIMETERS_PER_METER,
                    command.angVel
            );

            rrDrive.setDriveChassisSpeeds(robotCmd);

            telemetry.addData("target pose", RRAdapterUtil.toCommonPose(targetPose));
            telemetry.addData("pose", fieldPose);
            telemetry.addData("elapsed", elapsed);
            telemetry.update();

            idle();
        }

        rrDrive.setDriveChassisSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    private static PoseVelocity2d toRoadRunnerVelocity(ChassisSpeeds speeds)
    {
        ChassisSpeeds robotSpeeds = speeds;
        if (robotSpeeds == null)
        {
            robotSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        }

        return new PoseVelocity2d(
                new Vector2d(
                        robotSpeeds.vx / MILLIMETERS_PER_METER,
                        robotSpeeds.vy / MILLIMETERS_PER_METER
                ),
                robotSpeeds.omega
        );
    }
}
