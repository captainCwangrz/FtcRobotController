package org.firstinspires.ftc.team28770_SYSNG.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.common.drive.DriveIO;
import org.firstinspires.ftc.common.drive.WheelSpeeds;
import org.firstinspires.ftc.team28770_SYSNG.Team28770Constants;

/**
 * Team 28770 implementation of DriveIO for a 4-mecanum drivetrain.
 *
 * - Converts desired wheel speeds (mm/s) from common layer
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

        // Direction fixes ONLY live here.
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        configureMotor(fl);
        configureMotor(fr);
        configureMotor(bl);
        configureMotor(br);
    }

    @Override
    public void setWheelSpeeds(WheelSpeeds speeds)
    {
        fl.setVelocity(mmPerSecToTicksPerSec(speeds.fl));
        fr.setVelocity(mmPerSecToTicksPerSec(speeds.fr));
        bl.setVelocity(mmPerSecToTicksPerSec(speeds.bl));
        br.setVelocity(mmPerSecToTicksPerSec(speeds.br));
    }

    @Override
    public WheelSpeeds getWheelPositions()
    {
        return new WheelSpeeds(
                ticksToMm(fl.getCurrentPosition()),
                ticksToMm(fr.getCurrentPosition()),
                ticksToMm(bl.getCurrentPosition()),
                ticksToMm(br.getCurrentPosition())
        );
    }

    @Override
    public WheelSpeeds getWheelVelocities()
    {
        // If encoders valid: convert ticks/s back to mm/s
        return new WheelSpeeds(
                ticksPerSecToMmPerSec(fl.getVelocity()),
                ticksPerSecToMmPerSec(fr.getVelocity()),
                ticksPerSecToMmPerSec(bl.getVelocity()),
                ticksPerSecToMmPerSec(br.getVelocity())
        );
    }

    // --- Helpers ---

    private static void configureMotor(DcMotorEx motor)
    {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private static double mmPerSecToTicksPerSec(double mmPerSec)
    {
        if (!Double.isFinite(mmPerSec))
        {
            return 0.0;
        }
        double revPerSec = mmPerSec / Team28770Constants.WHEEL_CIRCUMFERENCE_MM;
        double motorRevPerSec = revPerSec / Team28770Constants.GEAR_RATIO;
        return motorRevPerSec * Team28770Constants.TICKS_PER_REV;
    }

    private static double ticksPerSecToMmPerSec(double ticksPerSec)
    {
        if (!Double.isFinite(ticksPerSec))
        {
            return 0.0;
        }
        double motorRevPerSec = ticksPerSec / Team28770Constants.TICKS_PER_REV;
        double wheelRevPerSec = motorRevPerSec * Team28770Constants.GEAR_RATIO;
        return wheelRevPerSec * Team28770Constants.WHEEL_CIRCUMFERENCE_MM;
    }

    private static double ticksToMm(double ticks)
    {
        if (!Double.isFinite(ticks))
        {
            return 0.0;
        }
        double motorRevolutions = ticks / Team28770Constants.TICKS_PER_REV;
        double wheelRevolutions = motorRevolutions * Team28770Constants.GEAR_RATIO;
        return wheelRevolutions * Team28770Constants.WHEEL_CIRCUMFERENCE_MM;
    }
}
