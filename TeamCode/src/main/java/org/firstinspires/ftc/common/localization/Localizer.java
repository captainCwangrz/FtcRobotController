package org.firstinspires.ftc.common.localization;

import org.firstinspires.ftc.common.geometry.Pose2d;

/**
 * Generic robot pose estimator.
 *
 * <p>Implementations may use dead wheels, IMU fusion, vision, or any other hardware. The
 * contract simply guarantees that callers can poll the current pose expressed in the
 * <strong>field frame</strong> using millimetres and radians.</p>
 */
public interface Localizer
{
    /**
     * Returns the current estimated robot pose in the <strong>field frame</strong>.
     *
     * @return pose in millimetres (x/y) and radians (heading)
     */
    Pose2d getPose();

    /**
     * Manually set/reset the estimated pose.
     * Useful for:
     *  - initializing to known start position in Auto
     *  - zeroing between matches
     *  - synchronizing with external localization (e.g. vision)
     */
    void setPose(Pose2d pose);

    /**
     * Update internal state.
     * Call once per loop (or at a fixed rate) from OpMode.
     */
    void update();
}
