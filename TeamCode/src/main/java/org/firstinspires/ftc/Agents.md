# agents.md — FTC Control Stack v2 (for Codex)

**Audience:** Codex + engineers.

This file is the **canonical spec** for how to generate, modify, and review the control stack code in this repo.

Codex must treat this document as ground truth:

* Obey the architecture.
* Obey units and coordinate conventions.
* Do not re-introduce “god classes” or SDK leakage into `common/**`.
* Integrate RoadRunner via adapters only.

Everything below is written as **instructions to Codex**.

---

## 0. Codex Operating Rules

When generating or editing code in this project, Codex must:

1. **Preserve the layered architecture** defined here.
2. **Never** import FTC SDK types (`DcMotorEx`, `OpMode`, etc.) inside `common/**`.
3. **Never** call hardware APIs outside team-specific layers or concrete adapters.
4. **Use `ChassisSpeeds` as the only chassis-level velocity type** and `WheelSpeeds` / `WheelPositions` as the only wheel-level types.
5. **Use one consistent linear unit `U`** across all code (meters or millimeters), as defined by the project; do not mix.
6. **Keep RoadRunner isolated** in `common/rr` and team-level Auto code; do not let RR grab motors directly.
7. **Prefer small, composable classes** (PID, helpers, adapters) over large multi-responsibility classes.

If existing code violates this spec, Codex should:

* Refactor toward this architecture.
* Preserve behavior where possible while moving logic into correct layers.

---

## 1. Global Conventions

### 1.1 Units

* Single linear unit `U` for all distances.
* Velocity: `U/s`
* Angle: radians in `(-π, π]`
* Angular velocity: `rad/s`

Codex:

* Ensure all comments, constants, and conversions are consistent with the chosen `U`.
* Do not silently mix inches/mm/m; always convert at the boundary and document.

### 1.2 Coordinate Frames

* **Field frame (F):**

  * +x: field forward
  * +y: field left
  * Heading/yaw: CCW positive, 0 along +x

* **Robot frame (R):**

  * +x: robot forward
  * +y: robot left
  * Heading/yaw: same convention; robot heading is its orientation in field frame.

Codex:

* Do not invent alternative conventions.
* If a function uses a specific frame, document it explicitly in Javadoc.

### 1.3 Wheel Order

* Fixed order: `fl, fr, bl, br`
* All kinematics, IO, and team code must respect this ordering.

---

## 2. Directory & Layering

Expected high-level layout:

```text
common/
  geometry/           # pure math & frames
  drive/              # chassis <-> wheels, DriveIO abstraction
  localization/       # pose estimation interfaces & impls
  control/            # TeleOp & Auto control algorithms (NEW)
  rr/                 # RoadRunner adapter / type bridge (NEW)

team28770_SYSNG/
  drive/              # DriveIO concrete implementation (uses SDK)
  opmode/             # TeleOp & Auto; composition only
  Team28770Constants  # robot-specific constants in U
```

Layer responsibilities:

* `common/geometry`: math, poses, vectors, transforms.
* `common/drive`: `ChassisSpeeds`, `WheelSpeeds`, `MecanumKinematics`, `MecanumDrive`, `DriveIO` interface.
* `common/localization`: `Localizer` interface + SDK-bound impls (Pinpoint, etc.).
* `common/control`: PID, slew, input shaping, heading hold, go-to-pose, holonomic controllers.
* `common/rr`: adapters to RoadRunner types & followers.
* `team...`: actual hardware bindings & OpModes.

Codex must keep each concern in its layer.

---

## 3. `common/geometry` Specification

Use existing files but align to this spec.

### 3.1 `Pose2d`

Semantics: robot pose in **field frame**.

```java
package org.firstinspires.ftc.common.geometry;

public final class Pose2d {
    public final double x;       // U
    public final double y;       // U
    public final double heading; // rad, (-π, π]

    public Pose2d(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = MathUtil.wrapAngle(heading);
    }

    public Vector2d vec() {
        return new Vector2d(x, y);
    }

    public Pose2d plus(Vector2d d) {
        return new Pose2d(x + d.x, y + d.y, heading);
    }

    public Pose2d withHeading(double newHeading) {
        return new Pose2d(x, y, newHeading);
    }
}
```

### 3.2 `Vector2d`

```java
package org.firstinspires.ftc.common.geometry;

public final class Vector2d {
    public final double x, y;

    public Vector2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2d plus(Vector2d o)    { return new Vector2d(x + o.x, y + o.y); }
    public Vector2d minus(Vector2d o)   { return new Vector2d(x - o.x, y - o.y); }
    public Vector2d times(double k)     { return new Vector2d(k * x, k * y); }
    public double   dot(Vector2d o)     { return x * o.x + y * o.y; }
    public double   norm()              { return Math.hypot(x, y); }
    public Vector2d normalized()        {
        double n = norm();
        return n > 0 ? times(1.0 / n) : new Vector2d(0.0, 0.0);
    }
    public Vector2d rotate(double a)    {
        double c = Math.cos(a), s = Math.sin(a);
        return new Vector2d(c * x - s * y, s * x + c * y);
    }
}
```

### 3.3 `MathUtil`

```java
package org.firstinspires.ftc.common.geometry;

public final class MathUtil {
    private MathUtil() {}

    public static double wrapAngle(double a) {
        double x = (a + Math.PI) % (2.0 * Math.PI);
        if (x < 0.0) x += 2.0 * Math.PI;
        return x - Math.PI;
    }

    public static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    public static boolean epsilonEquals(double a, double b, double eps) {
        return Math.abs(a - b) <= eps;
    }
}
```

### 3.4 `FrameTransform`

```java
package org.firstinspires.ftc.common.geometry;

import org.firstinspires.ftc.common.drive.ChassisSpeeds;

public final class FrameTransform {
    private FrameTransform() {}

    // Interpret vx,vy as field-relative; output robot-relative.
    public static ChassisSpeeds fieldToRobot(ChassisSpeeds f, double heading) {
        double c = Math.cos(heading), s = Math.sin(heading);
        double vxR =  c * f.vx + s * f.vy;
        double vyR = -s * f.vx + c * f.vy;
        return new ChassisSpeeds(vxR, vyR, f.omega);
    }

    // Interpret vx,vy as robot-relative; output field-relative.
    public static ChassisSpeeds robotToField(ChassisSpeeds r, double heading) {
        double c = Math.cos(heading), s = Math.sin(heading);
        double vxF =  c * r.vx - s * r.vy;
        double vyF =  s * r.vx + c * r.vy;
        return new ChassisSpeeds(vxF, vyF, r.omega);
    }
}
```

---

## 4. `common/drive` Specification

### 4.1 `ChassisSpeeds`

```java
package org.firstinspires.ftc.common.drive;

// Robot-frame chassis velocity
public final class ChassisSpeeds {
    public final double vx;    // U/s, +x forward (robot)
    public final double vy;    // U/s, +y left   (robot)
    public final double omega; // rad/s, CCW+

    public ChassisSpeeds(double vx, double vy, double omega) {
        this.vx = vx;
        this.vy = vy;
        this.omega = omega;
    }
}
```

### 4.2 `WheelSpeeds`

```java
package org.firstinspires.ftc.common.drive;

// Wheel linear speeds along drive direction
public final class WheelSpeeds {
    public double fl, fr, bl, br; // U/s

    public WheelSpeeds(double fl, double fr, double bl, double br) {
        this.fl = fl; this.fr = fr; this.bl = bl; this.br = br;
    }

    public WheelSpeeds times(double k) {
        return new WheelSpeeds(k * fl, k * fr, k * bl, k * br);
    }

    // Mutates this instance: scales down to maxMagnitude if needed.
    public void normalize(double maxMagnitude) {
        double m = maxAbs();
        if (m > maxMagnitude && m > 0.0) {
            double s = maxMagnitude / m;
            fl *= s; fr *= s; bl *= s; br *= s;
        }
    }

    public double maxAbs() {
        return Math.max(
            Math.max(Math.abs(fl), Math.abs(fr)),
            Math.max(Math.abs(bl), Math.abs(br))
        );
    }
}
```

### 4.3 `DriveIO`

```java
package org.firstinspires.ftc.common.drive;

// Abstract hardware bridge for the drivetrain.
public interface DriveIO {

    // Desired wheel linear speeds in U/s.
    void setWheelSpeeds(WheelSpeeds speeds);

    // Optional: direct power control for debug or simple TeleOp.
    default void setWheelPowers(WheelSpeeds powers) {
        throw new UnsupportedOperationException("setWheelPowers not implemented");
    }

    // Wheel positions in U (e.g., from encoders).
    WheelSpeeds getWheelPositions();

    // Wheel velocities in U/s.
    WheelSpeeds getWheelVelocities();
}
```

Codex:

* Implementation classes in `team.../drive` must:

  * Handle ticks↔U conversion.
  * Configure directions, zero-power behavior, run modes.
  * Not contain TeleOp logic or RR logic.

### 4.4 `MecanumKinematics`

```java
package org.firstinspires.ftc.common.drive;

// Standard rectangular mecanum kinematics.
public final class MecanumKinematics {
    private final double k; // (wheelBase + trackWidth) / 2, in U

    public MecanumKinematics(double wheelBase, double trackWidth) {
        this.k = 0.5 * (wheelBase + trackWidth);
    }

    public WheelSpeeds toWheelSpeeds(ChassisSpeeds c) {
        double fl =  c.vx - c.vy - c.omega * k;
        double fr =  c.vx + c.vy + c.omega * k;
        double bl =  c.vx + c.vy - c.omega * k;
        double br =  c.vx - c.vy + c.omega * k;
        return new WheelSpeeds(fl, fr, bl, br);
    }

    public ChassisSpeeds toChassisSpeeds(WheelSpeeds w) {
        double vx    = (w.fl + w.fr + w.bl + w.br) / 4.0;
        double vy    = (-w.fl + w.fr + w.bl - w.br) / 4.0;
        double omega = (-w.fl + w.fr - w.bl + w.br) / (4.0 * k);
        return new ChassisSpeeds(vx, vy, omega);
    }
}
```

Codex:

* Do not change the wheel order or sign conventions.
* If robot geometry changes, adjust constants in team constants, not here.

### 4.5 `MecanumDrive`

```java
package org.firstinspires.ftc.common.drive;

// Thin wrapper: chassis speeds -> wheel speeds -> IO
public final class MecanumDrive {
    private final MecanumKinematics kinematics;
    private final DriveIO io;
    private final double maxWheelSpeed; // U/s

    public MecanumDrive(MecanumKinematics kinematics, DriveIO io, double maxWheelSpeed) {
        this.kinematics = kinematics;
        this.io = io;
        this.maxWheelSpeed = maxWheelSpeed;
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        WheelSpeeds ws = kinematics.toWheelSpeeds(speeds);
        ws.normalize(maxWheelSpeed);
        io.setWheelSpeeds(ws);
    }

    public void stop() {
        io.setWheelSpeeds(new WheelSpeeds(0.0, 0.0, 0.0, 0.0));
    }

    public WheelSpeeds getMeasuredWheelSpeeds() {
        return io.getWheelVelocities();
    }
}
```

Codex:

* Do not add input shaping, PID, field-centric, or gamepad logic here.

---

## 5. `common/localization` Specification

### 5.1 `Localizer`

```java
package org.firstinspires.ftc.common.localization;

import org.firstinspires.ftc.common.geometry.Pose2d;

public interface Localizer {
    Pose2d getPose();          // field frame
    void setPose(Pose2d pose); // reset
    void update();             // call each loop
}
```

### 5.2 Concrete Localizers

* E.g. `PinpointLocalizer`:

  * May use FTC SDK / vendor libs internally.
  * Exposed API: `implements Localizer`, plus optional:

    * `ChassisSpeeds getRobotVelocity()`
    * `void resetAll()`
  * No gamepad or OpMode logic inside.

Codex:

* Keep SDK imports inside these concrete classes only.

---

## 6. `common/control` Specification (NEW)

This package hosts all reusable control logic.

Codex must implement these templates (extend as needed, but keep them focused).

### 6.1 `PIDController`

```java
package org.firstinspires.ftc.common.control;

import org.firstinspires.ftc.common.geometry.MathUtil;

public final class PIDController {
    public double kP, kI, kD;

    private boolean continuous = false;
    private double minInput, maxInput;
    private double setpoint;
    private double integral;
    private double prevError;
    private boolean first = true;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP; this.kI = kI; this.kD = kD;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void enableContinuousInput(double minInput, double maxInput) {
        this.continuous = true;
        this.minInput = minInput;
        this.maxInput = maxInput;
    }

    public void disableContinuousInput() {
        this.continuous = false;
    }

    private double getError(double measurement) {
        double error = setpoint - measurement;
        if (continuous) {
            double range = maxInput - minInput;
            error = (error + 0.5 * range) % range;
            if (error < 0) error += range;
            error -= 0.5 * range;
        }
        return error;
    }

    public double calculate(double measurement, double dt) {
        double error = getError(measurement);
        if (first) {
            prevError = error;
            first = false;
        }
        integral += error * dt;
        double derivative = (dt > 0) ? (error - prevError) / dt : 0.0;
        prevError = error;
        return kP * error + kI * integral + kD * derivative;
    }

    public void reset(double measurement) {
        first = true;
        integral = 0.0;
        prevError = getError(measurement);
    }
}
```

### 6.2 `SlewRateLimiter`

```java
package org.firstinspires.ftc.common.control;

public final class SlewRateLimiter {
    private final double rateLimit; // units per second
    private double lastVal;
    private double lastTime;
    private boolean first = true;

    public SlewRateLimiter(double rateLimit) {
        this.rateLimit = rateLimit;
    }

    public double calculate(double input, double timeSeconds) {
        if (first) {
            first = false;
            lastVal = input;
            lastTime = timeSeconds;
            return input;
        }
        double dt = timeSeconds - lastTime;
        lastTime = timeSeconds;
        if (dt <= 0) return lastVal;
        double maxDelta = rateLimit * dt;
        double delta = input - lastVal;
        if (delta > maxDelta) delta = maxDelta;
        if (delta < -maxDelta) delta = -maxDelta;
        lastVal += delta;
        return lastVal;
    }

    public void reset(double value, double timeSeconds) {
        first = false;
        lastVal = value;
        lastTime = timeSeconds;
    }
}
```

### 6.3 `TeleOpConfig` & `InputShaping`

```java
package org.firstinspires.ftc.common.control;

public record TeleOpConfig(
    double deadband,
    double expoTranslation,
    double expoRotation,
    double maxVel,      // U/s
    double maxAngVel    // rad/s
) {}
```

```java
package org.firstinspires.ftc.common.control;

public final class InputShaping {
    private InputShaping(){}

    public static double applyDeadband(double x, double db) {
        return (Math.abs(x) < db) ? 0.0 : x;
    }

    // expo in [0,1]; 0 linear, 1 strong curve
    public static double applyExpo(double x, double expo) {
        double sign = Math.signum(x);
        double ax = Math.abs(x);
        double curved = Math.pow(ax, 1.0 + expo * 2.0);
        return sign * curved;
    }

    public static double scale(double x, double maxAbs) {
        return x * maxAbs;
    }
}
```

### 6.4 `TeleOpDriveHelper`

```java
package org.firstinspires.ftc.common.control;

import org.firstinspires.ftc.common.drive.ChassisSpeeds;
import org.firstinspires.ftc.common.geometry.FrameTransform;

public final class TeleOpDriveHelper {
    private TeleOpDriveHelper(){}

    public static ChassisSpeeds robotRelativeFromJoysticks(
        double lx, double ly, double rx,
        TeleOpConfig cfg
    ) {
        double x = InputShaping.applyDeadband(-ly, cfg.deadband()); // stick forward = -ly
        double y = InputShaping.applyDeadband(-lx, cfg.deadband());
        double r = InputShaping.applyDeadband(-rx, cfg.deadband());

        x = InputShaping.applyExpo(x, cfg.expoTranslation());
        y = InputShaping.applyExpo(y, cfg.expoTranslation());
        r = InputShaping.applyExpo(r, cfg.expoRotation());

        double vx    = InputShaping.scale(x, cfg.maxVel());
        double vy    = InputShaping.scale(y, cfg.maxVel());
        double omega = InputShaping.scale(r, cfg.maxAngVel());

        return new ChassisSpeeds(vx, vy, omega);
    }

    public static ChassisSpeeds fieldRelativeFromJoysticks(
        double lx, double ly, double rx,
        double robotHeading,
        TeleOpConfig cfg
    ) {
        ChassisSpeeds robotRel =
            robotRelativeFromJoysticks(lx, ly, rx, cfg);
        // interpret current (vx,vy) as field and convert back? No:
        // better: treat stick as field frame, then rotate to robot.
        // For simplicity: reuse robotRelative and then rotate out if needed.
        // Here we actually want: joystick -> field -> robot:
        // So override: compute field vx,vy then FrameTransform.fieldToRobot.
        double x = InputShaping.applyDeadband(-ly, cfg.deadband());
        double y = InputShaping.applyDeadband(-lx, cfg.deadband());
        double r = InputShaping.applyDeadband(-rx, cfg.deadband());

        x = InputShaping.applyExpo(x, cfg.expoTranslation());
        y = InputShaping.applyExpo(y, cfg.expoTranslation());
        r = InputShaping.applyExpo(r, cfg.expoRotation());

        double vxF   = InputShaping.scale(x, cfg.maxVel());
        double vyF   = InputShaping.scale(y, cfg.maxVel());
        double omega = InputShaping.scale(r, cfg.maxAngVel());

        return FrameTransform.fieldToRobot(
            new ChassisSpeeds(vxF, vyF, omega),
            robotHeading
        );
    }
}
```

Codex:

* TeleOp code in team package should call these helpers instead of hardcoding curves.

### 6.5 `HeadingController`

```java
package org.firstinspires.ftc.common.control;

import org.firstinspires.ftc.common.geometry.MathUtil;

public final class HeadingController {
    private final PIDController pid;

    public HeadingController(PIDController pid) {
        this.pid = pid;
        this.pid.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void setTarget(double headingRad) {
        pid.setSetpoint(MathUtil.wrapAngle(headingRad));
    }

    public double update(double currentHeading, double dt) {
        return pid.calculate(MathUtil.wrapAngle(currentHeading), dt);
    }
}
```

### 6.6 `GoToPoseController`

```java
package org.firstinspires.ftc.common.control;

import org.firstinspires.ftc.common.geometry.Pose2d;
import org.firstinspires.ftc.common.geometry.Vector2d;
import org.firstinspires.ftc.common.drive.ChassisSpeeds;

public record GoToPoseConfig(
    double maxVel,
    double maxAngVel
) {}

public final class GoToPoseController {
    private final PIDController xPid, yPid, thetaPid;
    private final GoToPoseConfig cfg;

    public GoToPoseController(PIDController xPid, PIDController yPid,
                              PIDController thetaPid, GoToPoseConfig cfg) {
        this.xPid = xPid;
        this.yPid = yPid;
        this.thetaPid = thetaPid;
        this.thetaPid.enableContinuousInput(-Math.PI, Math.PI);
        this.cfg = cfg;
    }

    public ChassisSpeeds calculate(Pose2d current, Pose2d target, double dt) {
        // Error in field frame
        double ex = target.x - current.x;
        double ey = target.y - current.y;

        // For simplicity, treat directly as field-frame control:
        double vxCmd = xPid.calculate(current.x, dt);
        double vyCmd = yPid.calculate(current.y, dt);

        // But we want: setpoints are target.x/y
        xPid.setSetpoint(target.x);
        yPid.setSetpoint(target.y);
        thetaPid.setSetpoint(target.heading);

        vxCmd = xPid.calculate(current.x, dt);
        vyCmd = yPid.calculate(current.y, dt);
        double omegaCmd = thetaPid.calculate(current.heading, dt);

        // Clamp
        double vMag = Math.hypot(vxCmd, vyCmd);
        if (vMag > cfg.maxVel() && vMag > 0) {
            double s = cfg.maxVel() / vMag;
            vxCmd *= s;
            vyCmd *= s;
        }
        if (Math.abs(omegaCmd) > cfg.maxAngVel()) {
            omegaCmd = Math.copySign(cfg.maxAngVel(), omegaCmd);
        }

        // Returned as field-relative here; caller may convert as needed,
        // or this could be adapted to robot-frame directly.
        return new ChassisSpeeds(vxCmd, vyCmd, omegaCmd);
    }
}
```

(Teams can refine; Codex should keep structure and frame usage consistent.)

### 6.7 `HolonomicDriveController`

```java
package org.firstinspires.ftc.common.control;

import org.firstinspires.ftc.common.drive.ChassisSpeeds;
import org.firstinspires.ftc.common.geometry.Pose2d;
import org.firstinspires.ftc.common.geometry.MathUtil;

public final class HolonomicDriveController {
    private final PIDController xPid, yPid, thetaPid;

    public HolonomicDriveController(PIDController xPid, PIDController yPid, PIDController thetaPid) {
        this.xPid = xPid;
        this.yPid = yPid;
        this.thetaPid = thetaPid;
        this.thetaPid.enableContinuousInput(-Math.PI, Math.PI);
    }

    public ChassisSpeeds calculate(
        Pose2d current,
        Pose2d referencePose,
        ChassisSpeeds referenceVel,
        double dt
    ) {
        xPid.setSetpoint(referencePose.x);
        yPid.setSetpoint(referencePose.y);
        thetaPid.setSetpoint(referencePose.heading);

        double vxFF = referenceVel.vx;
        double vyFF = referenceVel.vy;
        double omegaFF = referenceVel.omega;

        double vxFB = xPid.calculate(current.x, dt);
        double vyFB = yPid.calculate(current.y, dt);
        double omegaFB = thetaPid.calculate(current.heading, dt);

        double vxCmd = vxFF + vxFB;
        double vyCmd = vyFF + vyFB;
        double omegaCmd = omegaFF + omegaFB;

        return new ChassisSpeeds(vxCmd, vyCmd, omegaCmd);
    }
}
```

Codex:

* Use this in Auto to track reference trajectories (including RR-generated).

---

## 7. `common/rr` Specification (NEW)

Codex must use this package to integrate RoadRunner.

### 7.1 `RRAdapterUtil`

```java
package org.firstinspires.ftc.common.rr;

import org.firstinspires.ftc.common.geometry.Pose2d;

public final class RRAdapterUtil {
    private RRAdapterUtil(){}

    public static com.acmerobotics.roadrunner.geometry.Pose2d toRR(Pose2d p) {
        return new com.acmerobotics.roadrunner.geometry.Pose2d(p.x, p.y, p.heading);
    }

    public static Pose2d fromRR(com.acmerobotics.roadrunner.geometry.Pose2d p) {
        return new Pose2d(p.getX(), p.getY(), p.getHeading());
    }
}
```

### 7.2 `RRHolonomicDriveAdapter`

Used by RR follower to drive using our stack.

```java
package org.firstinspires.ftc.common.rr;

import org.firstinspires.ftc.common.drive.ChassisSpeeds;
import org.firstinspires.ftc.common.drive.MecanumDrive;
import org.firstinspires.ftc.common.localization.Localizer;
import org.firstinspires.ftc.common.geometry.Pose2d;

public final class RRHolonomicDriveAdapter {
    private final MecanumDrive drive;
    private final Localizer localizer;

    public RRHolonomicDriveAdapter(MecanumDrive drive, Localizer localizer) {
        this.drive = drive;
        this.localizer = localizer;
    }

    public void setDriveChassisSpeeds(ChassisSpeeds speeds) {
        drive.driveRobotRelative(speeds);
    }

    public Pose2d getPoseEstimate() {
        return localizer.getPose();
    }
}
```

Codex:

* RR-based Auto OpModes should:

  * Use RR to build trajectories.
  * Use RR follower that outputs desired chassis motion.
  * Adapt through this layer into `MecanumDrive`.
  * Never hold `DcMotorEx` directly inside RR code.

---

## 8. `team28770_SYSNG` Guidelines

Codex must align team code to this spec.

### 8.1 `Team28770Constants`

* Store:

  * Motor names.
  * `WHEEL_RADIUS`, `WHEEL_BASE`, `TRACK_WIDTH`.
  * `TICKS_PER_REV`, `GEAR_RATIO`.
  * `MAX_WHEEL_SPEED` (U/s), `TELEOP_MAX_VEL`, `TELEOP_MAX_ANG_VEL`.
  * Pinpoint / odometry geometry.
* All in unit `U`.

### 8.2 `Team28770MecanumDriveIO`

Implements `DriveIO`:

* In constructor:

  * Fetch motors from `hardwareMap`.
  * Set directions, BRAKE, RunMode.
* `setWheelSpeeds`:

  * Convert from U/s → ticks/s or power via known constants.
* `getWheelPositions` / `getWheelVelocities`:

  * Convert from ticks → U / U/s.

No gamepad, PID, or RR logic here.

### 8.3 TeleOp

TeleOp OpMode should:

* Construct:

  * `Team28770MecanumDriveIO`
  * `MecanumKinematics`
  * `MecanumDrive`
  * `Localizer`
  * TeleOp control helpers (`TeleOpConfig`, `HeadingController`, `SlewRateLimiter`, etc.)

Loop sketch:

```java
localizer.update();
Pose2d pose = localizer.getPose();

ChassisSpeeds cmd = TeleOpDriveHelper.fieldRelativeFromJoysticks(
    gamepad1.left_stick_x,
    gamepad1.left_stick_y,
    gamepad1.right_stick_x,
    pose.heading,
    teleOpConfig
);

// Optional: integrate HeadingController / SlewRateLimiter before drive
drive.driveRobotRelative(cmd);
```

### 8.4 Auto with RoadRunner

Auto OpMode should:

* Build RR trajectories (RR APIs).
* Each loop:

  * `localizer.update()`
  * Use RR follower to compute desired command (preferably `ChassisSpeeds`).
  * Call `drive.driveRobotRelative(cmd)` via `RRHolonomicDriveAdapter` or equivalent.

No direct motor writes.

---

## 9. Things Codex Must Not Do

* Do not:

  * Add FTC SDK imports into `common/**` (except concrete localizers/bridges explicitly allowed).
  * Recreate `SampleMecanumDrive`-style god classes that mix:

    * hardware
    * localization
    * control
    * trajectories
  * Scatter units conversion or magic scale factors across files.
  * Hardcode wheel order inconsistently.
  * Implement TeleOp logic inside `DriveIO`, `MecanumDrive`, `Localizer`, or `common/rr`.

If existing code does these:

* Refactor progressively toward this spec.

---

This `agents.md` is the reference playbook.
Codex should use it as the mental wiring diagram whenever generating or editing code in this repo.
