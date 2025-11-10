package org.firstinspires.ftc.common.rr;

import java.util.Objects;

import org.firstinspires.ftc.common.drive.ChassisSpeeds;
import org.firstinspires.ftc.common.drive.MecanumDrive;
import org.firstinspires.ftc.common.geometry.Pose2d;
import org.firstinspires.ftc.common.localization.Localizer;

public class RRHolonomicDriveAdapter
{
    private final MecanumDrive drive;
    private final Localizer localizer;

    public RRHolonomicDriveAdapter(MecanumDrive drive, Localizer localizer)
    {
        this.drive = Objects.requireNonNull(drive, "drive");
        this.localizer = Objects.requireNonNull(localizer, "localizer");
    }

    public void setDriveChassisSpeeds(ChassisSpeeds speeds)
    {
        drive.driveRobotRelative(Objects.requireNonNull(speeds, "speeds"));
    }

    public Pose2d getPoseEstimate()
    {
        return localizer.getPose();
    }
}
