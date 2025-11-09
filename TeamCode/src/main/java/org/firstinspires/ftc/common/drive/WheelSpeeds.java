package org.firstinspires.ftc.common.drive;

/**
 * Linear speeds of a 4-mecanum wheel drivetrain.
 *
 * Units:
 *  - All fields are linear wheel speeds along the driving direction (mm/s).
 *
 * Wheel order convention:
 *  - fl: front-left
 *  - fr: front-right
 *  - bl: back-left
 *  - br: back-right
 *
 * This is a math container, not tied to specific motor IDs.
 */
public class WheelSpeeds
{
    public double fl;
    public double fr;
    public double bl;
    public double br;

    public WheelSpeeds(double fl, double fr, double bl, double br)
    {
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;
    }

    /**
     * Scale all wheel speeds by a factor.
     */
    public WheelSpeeds times(double s)
    {
        return new WheelSpeeds(fl * s, fr * s, bl * s, br * s);
    }

    /**
     * If any magnitude exceeds max, scale all down proportionally to fit.
     * Useful for normalizing commands to within allowed wheel speed.
     */
    public void normalize(double maxMagnitude)
    {
        double max = Math.max(
                Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))
        );
        if (max > maxMagnitude && max > 1e-9)
        {
            double scale = maxMagnitude / max;
            fl *= scale;
            fr *= scale;
            bl *= scale;
            br *= scale;
        }
    }

    @Override
    public String toString()
    {
        return String.format("WheelSpeeds(fl=%.4f, fr=%.4f, bl=%.4f, br=%.4f)", fl, fr, bl, br);
    }
}
