package org.firstinspires.ftc.common.localization;

import org.firstinspires.ftc.common.geometry.Pose2d;

/**
 * Generic robot pose estimator.
 *
 * Implementations may use encoders, IMU, external sensors, etc.
 * All poses are expressed in the FIELD frame using the global convention.
 */
public interface Localizer
{
    /**
     * Get the current estimated robot pose in the FIELD frame.
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
