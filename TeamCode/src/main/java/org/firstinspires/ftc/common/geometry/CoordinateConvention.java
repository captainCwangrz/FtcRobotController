package org.firstinspires.ftc.common.geometry;

/**
 * Global coordinate, angle, and unit conventions for the entire codebase.
 *
 * Field frame (F):
 *  - Right-handed 2D.
 *  - +x: field "forward" direction (chosen per game & documented elsewhere).
 *  - +y: to the left when facing +x.
 *  - Heading θ_F: radians, counter-clockwise (CCW) positive, θ_F = 0 along +x.
 *
 * Robot frame (R):
 *  - Attached to the robot.
 *  - +x: robot forward.
 *  - +y: robot left.
 *  - Heading θ_R: radians, CCW positive; θ_R = 0 when robot forward aligns with field +x.
 *
 * Units:
 *  - Distances: millimeters (mm).
 *  - Linear velocities: mm/s.
 *  - Angles: radians in (-π, π].
 *  - Angular velocities: rad/s.
 *
 * Rules:
 *  - All core math/logic uses these conventions and units.
 *  - Any hardware/sensor/library using different units or axes must be adapted
 *    before entering common code.
 */
public final class CoordinateConvention
{
    private CoordinateConvention() {}
}