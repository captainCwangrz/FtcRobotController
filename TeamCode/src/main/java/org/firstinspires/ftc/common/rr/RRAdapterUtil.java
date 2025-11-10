package org.firstinspires.ftc.common.rr;

import java.util.Objects;

import org.firstinspires.ftc.common.geometry.Pose2d;

/**
 * Utility helpers for bridging Road Runner classes with the common geometry types.
 */
public final class RRAdapterUtil
{
    private static final double MILLIMETERS_PER_METER = 1000.0;

    private RRAdapterUtil()
    {
    }

    public static com.acmerobotics.roadrunner.geometry.Pose2d toRoadRunnerPose(Pose2d pose)
    {
        Objects.requireNonNull(pose, "pose");
        // TODO: confirm RR units
        return new com.acmerobotics.roadrunner.geometry.Pose2d(
            pose.x / MILLIMETERS_PER_METER,
            pose.y / MILLIMETERS_PER_METER,
            pose.heading
        );
    }

    public static Pose2d toCommonPose(com.acmerobotics.roadrunner.geometry.Pose2d pose)
    {
        Objects.requireNonNull(pose, "pose");
        // TODO: confirm RR units
        return new Pose2d(
            pose.getX() * MILLIMETERS_PER_METER,
            pose.getY() * MILLIMETERS_PER_METER,
            pose.getHeading()
        );
    }
}
