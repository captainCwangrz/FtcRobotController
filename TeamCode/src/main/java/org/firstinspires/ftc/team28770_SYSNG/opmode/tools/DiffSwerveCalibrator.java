package org.firstinspires.ftc.team28770_SYSNG.opmode.tools;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Disabled
@TeleOp(name="DiffSwerve_Calibrator", group="test")
public class DiffSwerveCalibrator extends LinearOpMode {

    // ===== Hardware =====
    private DcMotorEx leftA, leftB, rightA, rightB;

    // ===== Steering direction conventions =====
    // Positive command (stick right) should produce CW (top view). Flip signs if your build is opposite.
    private static final int STEER_CW_SUM_SIGN_L = +1; // set to -1 if CW is inverted on left module
    private static final int STEER_CW_SUM_SIGN_R = +1; // set to -1 if CW is inverted on right module

    // ===== Operator parameters =====
    private double maxSteerVelTps = 400.0;  // per-motor TPS at full stick deflection (safe & slow)
    private static final double FINE_SCALE = 0.18; // fine mode multiplier
    private int plannedRotations = 10;      // user-counted turns between START and END

    // ===== Per-module run state =====
    private boolean haveStartL = false, haveStartR = false;
    private int startSumL = 0, startSumR = 0;
    private boolean fineL = false, fineR = false;

    // Saved results: ticks per 360° per direction per module
    private double cwTicks360L = Double.NaN, ccwTicks360L = Double.NaN;
    private double cwTicks360R = Double.NaN, ccwTicks360R = Double.NaN;

    // Button edge state
    private boolean lastA, lastB, lastX, lastY, lastLB, lastRB, lastUp, lastDown, lastLeft, lastRight, lastLS, lastRS;

    @Override
    public void runOpMode() {
        // Map hardware
        leftA  = hw("leftA");  leftB  = hw("leftB");
        rightA = hw("rightA"); rightB = hw("rightB");

        for (DcMotorEx m : new DcMotorEx[]{leftA,leftB,rightA,rightB}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        telemetry.addLine("DiffSwerve SUM (steer) per-module manual calibrator");
        telemetry.addLine("Left steer:  LS-X    | Right steer: RS-X");
        telemetry.addLine("Fine: LS press (left), RS press (right)");
        telemetry.addLine("Left START/END:  A / B     | Right START/END:  X / Y");
        telemetry.addLine("LB: ZERO encoders & clear   | RB: STOP motors");
        telemetry.addLine("Dpad Up/Down: rotations ±1  | Left/Right: max TPS ±50");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Read inputs
            double lx = gamepad1.left_stick_x;    // [-1..1], left module steer
            double rx = gamepad1.right_stick_x;   // [-1..1], right module steer
            boolean a = gamepad1.a, b = gamepad1.b, x = gamepad1.x, y = gamepad1.y;
            boolean lb = gamepad1.left_bumper, rb = gamepad1.right_bumper;
            boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;
            boolean left = gamepad1.dpad_left, right = gamepad1.dpad_right;
            boolean ls = gamepad1.left_stick_button, rs = gamepad1.right_stick_button;

            // Fine toggles
            if (edge(ls, lastLS)) fineL = !fineL;
            if (edge(rs, lastRS)) fineR = !fineR;

            // Params
            if (edge(up, lastUp))    plannedRotations = Math.min(200, Math.max(1, plannedRotations + 1));
            if (edge(down, lastDown))plannedRotations = Math.min(200, Math.max(1, plannedRotations - 1));
            if (edge(right, lastRight)) maxSteerVelTps = Math.min(3000.0, maxSteerVelTps + 50.0);
            if (edge(left, lastLeft))   maxSteerVelTps = Math.max(50.0,   maxSteerVelTps - 50.0);

            // Zero & clear
            if (edge(lb, lastLB)) {
                stopMotors();
                for (DcMotorEx m : new DcMotorEx[]{leftA,leftB,rightA,rightB}) {
                    m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                haveStartL = haveStartR = false;
                cwTicks360L = ccwTicks360L = cwTicks360R = ccwTicks360R = Double.NaN;
            }

            // Panic stop
            if (edge(rb, lastRB)) stopMotors();

            // Mark START per module
            if (edge(a, lastA)) { startSumL = getSumL(); haveStartL = true; }
            if (edge(x, lastX)) { startSumR = getSumR(); haveStartR = true; }

            // Mark END/compute per module
            if (edge(b, lastB) && haveStartL && plannedRotations > 0) {
                int deltaL = getSumL() - startSumL;
                double t360 = Math.abs((double)deltaL) / plannedRotations;
                boolean wasCW = (Integer.signum(deltaL) * STEER_CW_SUM_SIGN_L) > 0;
                if (wasCW) cwTicks360L = t360; else ccwTicks360L = t360;
            }
            if (edge(y, lastY) && haveStartR && plannedRotations > 0) {
                int deltaR = getSumR() - startSumR;
                double t360 = Math.abs((double)deltaR) / plannedRotations;
                boolean wasCW = (Integer.signum(deltaR) * STEER_CW_SUM_SIGN_R) > 0;
                if (wasCW) cwTicks360R = t360; else ccwTicks360R = t360;
            }

            // Manual steering output (per module)
            double scaleL = fineL ? FINE_SCALE : 1.0;
            double scaleR = fineR ? FINE_SCALE : 1.0;

            double sL = lx * maxSteerVelTps * scaleL * STEER_CW_SUM_SIGN_L; // +X = CW
            double sR = rx * maxSteerVelTps * scaleR * STEER_CW_SUM_SIGN_R; // +X = CW

            // Set both motors same sign for pure SUM steering
            leftA.setVelocity(sL);  leftB.setVelocity(sL);
            rightA.setVelocity(sR); rightB.setVelocity(sR);

            // Live deltas (since START)
            int sumL = getSumL(), sumR = getSumR();
            double liveDeltaL = haveStartL ? Math.abs(sumL - startSumL) : Double.NaN;
            double liveDeltaR = haveStartR ? Math.abs(sumR - startSumR) : Double.NaN;

            double liveT360L = (haveStartL && plannedRotations>0) ? (liveDeltaL / plannedRotations) : Double.NaN;
            double liveT360R = (haveStartR && plannedRotations>0) ? (liveDeltaR / plannedRotations) : Double.NaN;

            // Averages & constants
            Double avgL = averaged(cwTicks360L, ccwTicks360L);
            Double avgR = averaged(cwTicks360R, ccwTicks360R);
            Double radPerSumTickL = (avgL != null) ? (2.0 * Math.PI / avgL) : null;
            Double radPerSumTickR = (avgR != null) ? (2.0 * Math.PI / avgR) : null;

            // Telemetry
            telemetry.addLine("=== SUM Cal (Per-Module Manual) ===");
            telemetry.addData("Rotations", plannedRotations);
            telemetry.addData("maxSteerVel (tps)", "%.0f", maxSteerVelTps);
            telemetry.addData("Fine L / R", "%s / %s", (fineL?"ON":"OFF"), (fineR?"ON":"OFF"));

            telemetry.addLine("--- Live ---");
            telemetry.addData("SumL / SumR", "%d / %d", sumL, sumR);
            telemetry.addData("Δticks since START L / R", "%.0f / %.0f", liveDeltaL, liveDeltaR);
            telemetry.addData("Live ticks/360  L / R", "%.1f / %.1f", liveT360L, liveT360R);
            telemetry.addData("Cmd steer tps   L / R", "%.0f / %.0f", sL, sR);

            telemetry.addLine("--- Saved (ticks/360) ---");
            telemetry.addData("CW   L / R", "%.1f / %.1f", cwTicks360L, cwTicks360R);
            telemetry.addData("CCW  L / R", "%.1f / %.1f", ccwTicks360L, ccwTicks360R);

            telemetry.addLine("--- Averaged & constants ---");
            telemetry.addData("AVG ticks/360   L / R", "%.1f / %.1f",
                    (avgL==null?Double.NaN:avgL), (avgR==null?Double.NaN:avgR));
            telemetry.addData("RAD_PER_SUMTICK L / R", "%.6f / %.6f",
                    (radPerSumTickL==null?Double.NaN:radPerSumTickL),
                    (radPerSumTickR==null?Double.NaN:radPerSumTickR));
            telemetry.update();

            // update edges
            lastA=a; lastB=b; lastX=x; lastY=y; lastLB=lb; lastRB=rb;
            lastUp=up; lastDown=down; lastLeft=left; lastRight=right; lastLS=ls; lastRS=rs;
        }

        stopMotors();
    }

    // ===== Helpers =====
    private DcMotorEx hw(String n){ return hardwareMap.get(DcMotorEx.class, n); }

    private boolean edge(boolean now, boolean last){ return now && !last; }

    private void stopMotors(){
        if (leftA!=null)  leftA.setVelocity(0);
        if (leftB!=null)  leftB.setVelocity(0);
        if (rightA!=null) rightA.setVelocity(0);
        if (rightB!=null) rightB.setVelocity(0);
    }

    private int getSumL(){ return leftA.getCurrentPosition() + leftB.getCurrentPosition(); }
    private int getSumR(){ return rightA.getCurrentPosition() + rightB.getCurrentPosition(); }

    private Double averaged(double cw, double ccw){
        boolean vcw = !Double.isNaN(cw);
        boolean vccw= !Double.isNaN(ccw);
        if (vcw && vccw) return 0.5*(cw+ccw);
        if (vcw) return cw;
        if (vccw) return ccw;
        return null;
    }
}
