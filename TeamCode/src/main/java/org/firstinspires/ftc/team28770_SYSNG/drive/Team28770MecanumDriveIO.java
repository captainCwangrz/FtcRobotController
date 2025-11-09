package org.firstinspires.ftc.team28770_SYSNG.drive;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.common.drive.DriveIO;
import org.firstinspires.ftc.common.drive.WheelSpeeds;
import org.firstinspires.ftc.team28770_SYSNG.Team28770Constants;

/**
 * Team 28770 implementation of DriveIO for a 4-mecanum drivetrain.
 *
 * - Converts desired wheel speeds (m/s) from common layer
 *   into encoder ticks/s for REV DcMotorEx.
 */
public class Team28770MecanumDriveIO implements DriveIO
{
    private final DcMotorEx fl, fr, bl, br;

    public Team28770MecanumDriveIO(HardwareMap hw)
    {
        fl = hw.get(DcMotorEx.class, Team28770Constants.MOTOR_FL);
        fr = hw.get(DcMotorEx.class, Team28770Constants.MOTOR_FR);
        bl = hw.get(DcMotorEx.class, Team28770Constants.MOTOR_BL);
        br = hw.get(DcMotorEx.class, Team28770Constants.MOTOR_BR);

        //Direction fixes ONLY live here.
        //fl.setDirection(DcMotorEx.Direction.REVERSE);
        //fr.setDirection(DcMotorEx.Direction.REVERSE);
        //bl.setDirection(DcMotorEx.Direction.REVERSE);
        //br.setDirection(DcMotorEx.Direction.REVERSE);

        fl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void setWheelSpeeds(WheelSpeeds speeds)
    {
        fl.setVelocity(mpsToTicksPerSec(speeds.fl));
        fr.setVelocity(mpsToTicksPerSec(speeds.fr));
        bl.setVelocity(mpsToTicksPerSec(speeds.bl));
        br.setVelocity(mpsToTicksPerSec(speeds.br));
    }

    @Override
    public WheelSpeeds getWheelSpeeds()
    {
        // If encoders valid: convert ticks/s back to m/s
        return new WheelSpeeds(
                ticksPerSecToMps(fl.getVelocity()),
                ticksPerSecToMps(fr.getVelocity()),
                ticksPerSecToMps(bl.getVelocity()),
                ticksPerSecToMps(br.getVelocity())
        );
    }

    // --- Helpers ---

    private static double mpsToTicksPerSec(double mps)
    {
        double revPerSec = mps / Team28770Constants.WHEEL_CIRCUMFERENCE;
        double motorRevPerSec = revPerSec / Team28770Constants.GEAR_RATIO;
        return motorRevPerSec * Team28770Constants.TICKS_PER_REV;
    }

    private static double ticksPerSecToMps(double tps)
    {
        double motorRevPerSec = tps / Team28770Constants.TICKS_PER_REV;
        double wheelRevPerSec = motorRevPerSec * Team28770Constants.GEAR_RATIO;
        return wheelRevPerSec * Team28770Constants.WHEEL_CIRCUMFERENCE;
    }
}
