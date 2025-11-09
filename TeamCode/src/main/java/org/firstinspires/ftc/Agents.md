# Agents.md — Control Stack Ground Rules (For Codex)

This document is the **single source of truth** for how Codex must read, generate, and modify code in this repository.

Codex: follow this exactly. Where details are unknown, use **reasonable placeholders** and tag them with `// TODO: tune` or `// TODO: measure`.

---

## 1. Global Conventions

### 1.1 Units (Hard Requirement)

All core logic uses:

- Distance: **millimeters** (`mm`)
- Linear velocity: **mm/s**
- Angle: **radians** in `(-π, π]`
- Angular velocity: **rad/s**

Codex must:

- Use these units everywhere in `common/**`.
- Ensure comments and Javadocs explicitly match.
- Never silently mix inches, meters, or encoder ticks with mm.
- Do unit conversion **only** at adapter / hardware / RR integration boundaries, with comments.

### 1.2 Coordinate Frames

Two frames:

1. **Field frame (F):**
   - +x: field forward
   - +y: field left
   - Heading: CCW+, 0 along +x

2. **Robot frame (R):**
   - +x: robot forward
   - +y: robot left
   - Heading: CCW+

Rules:

- `Pose2d` in `common/geometry` represents robot pose in **field frame** (unless explicitly documented otherwise).
- `ChassisSpeeds` represents chassis velocity in **robot frame**.
- Transformations between frames live in `common/geometry` (e.g. `FrameTransform`), not scattered.

### 1.3 Wheel Order

Wheel order is globally fixed:

- `fl`: front-left
- `fr`: front-right
- `bl`: back-left
- `br`: back-right

All kinematics, IO, and team code must follow this order.

---

## 2. Package Structure & Responsibilities

Codex must preserve and respect this structure.

```text
common/
  geometry/           # Pure math & frames
  drive/              # Chassis <-> wheels, DriveIO abstraction
  localization/       # Pose estimation interfaces & impls
  control/            # TeleOp & Auto control algorithms
  rr/                 # RoadRunner adapters & unit bridging

team28770_SYSNG/
  drive/              # DriveIO concrete implementation (FTC SDK allowed)
  opmode/             # TeleOp & Auto; compose existing pieces
  Team28770Constants  # Robot-specific constants (in mm / mm/s / rad)
```

### 2.1 Dependency Rules

- `common/**`
  - No FTC SDK types (e.g. `DcMotorEx`, `OpMode`, `HardwareMap`), **except** inside clearly-marked hardware adapters or localizers where absolutely necessary.
- `team28770_SYSNG/**`
  - May use FTC SDK.
  - Must not reimplement generic math/kinematics/controls that belong in `common/**`.
- RoadRunner:
  - Must be integrated via `common/rr` and team-level Auto.
  - RoadRunner must **not** own or configure motors directly.

---

## 3. Core Abstractions (Codex Must Use)

### 3.1 `common/geometry`

**Key types (should already exist; Codex maintains/aligns):**

- `Pose2d`
  - Fields: `x` (mm), `y` (mm), `heading` (rad, wrapped to `(-π, π]`).
  - Default semantics: robot pose in field frame.
- `Vector2d`
  - Basic vector operations; treated as mm when used for positions.
- `MathUtil`
  - `wrapAngle`, `clamp`, `epsilonEquals`.
- `FrameTransform`
  - Functions for converting `ChassisSpeeds` between field and robot frames.

No hardware, no team logic, no RR logic here.

---

### 3.2 `common/drive`

**`ChassisSpeeds`**

- Robot-frame chassis velocity.
- Fields:
  - `vx`: mm/s, +x forward
  - `vy`: mm/s, +y left
  - `omega`: rad/s, CCW+

**`WheelSpeeds`**

- Wheel linear speeds in mm/s.
- Fields: `fl, fr, bl, br`.
- Utilities:
  - `times(double)`
  - `normalize(double maxMagnitude)` (mutating)
  - `maxAbs()`

**`DriveIO` (interface)**

The **only** abstraction allowed to touch drivetrain hardware (via implementations in `team...`):

```java
void setWheelSpeeds(WheelSpeeds speeds);    // input in mm/s

default void setWheelPowers(WheelSpeeds p)  // optional; may throw
WheelSpeeds getWheelPositions();            // output in mm
WheelSpeeds getWheelVelocities();           // output in mm/s
```

- No FTC types in the interface.
- Implementations in `team28770_SYSNG/drive`.

**`MecanumKinematics`**

- Rectangular mecanum kinematics.
- Constructor: `(wheelBaseMm, trackWidthMm)`.
- Methods:
  - `toWheelSpeeds(ChassisSpeeds)`
  - `toChassisSpeeds(WheelSpeeds)`
- Uses standard equations with wheel order `fl, fr, bl, br`.
- All distances in mm.

**`MecanumDrive`**

- Thin wrapper:
  - `driveRobotRelative(ChassisSpeeds)`:
    - Uses `MecanumKinematics` → `WheelSpeeds` → normalize → `DriveIO.setWheelSpeeds`.
  - `stop()`
  - `getMeasuredWheelSpeeds()`

Prohibited inside `MecanumDrive`:

- Gamepad logic
- TeleOp mode logic
- PID, filtering, trajectory following
- RoadRunner calls

---

### 3.3 `common/localization`

**`Localizer`**

```java
Pose2d getPose();    // pose in field frame (mm, rad)
void setPose(Pose2d pose);
void update();       // call every loop
```

**Concrete localizers** (e.g. `PinpointLocalizer`):

- May use FTC SDK / vendor APIs.
- Must output `Pose2d` in mm & rad.
- May expose helper methods (`getRobotVelocity`, `resetAll`).
- Must not read gamepads or contain TeleOp logic.

---

### 3.4 `common/control`

This package hosts reusable control algorithms.

Required components:

- `PIDController`
- `SlewRateLimiter`
- `TeleOpConfig`
- `InputShaping`
- `TeleOpDriveHelper`
- `HeadingController`
- `HolonomicDriveController`
- (Optional/simple) `GoToPoseController`

Units:

- Positions: mm
- Velocities: mm/s
- Angles: rad
- Angular rates: rad/s

Codex should provide reasonable default gains and limits as placeholders, always tagged `// TODO: tune`.

---

### 3.5 `common/rr`

Adapters for RoadRunner integration.

Responsibilities:

- Convert between project types and RR types.
- Handle unit scaling (mm ↔ RR units) in one place.
- Provide a thin adapter that lets RR followers command `ChassisSpeeds` into `MecanumDrive`.

Prohibited:

- Direct motor access.
- OpMode logic.
- Duplicating generic control layer functionality.

---

## 4. Team Layer (`team28770_SYSNG`)

**`Team28770Constants`**

- All robot-specific constants in mm / mm/s / rad.
- Used by:
  - `Team28770MecanumDriveIO`
  - OpModes
  - RR configs
- Codex may use safe placeholders; every placeholder must be `// TODO: tune` or `// TODO: measure`.

**`Team28770MecanumDriveIO`**

- Implements `DriveIO`:
  - Uses `Team28770Constants` for ticks↔mm.
  - Handles motor direction, modes, etc.
- No RR logic, no TeleOp logic.

**TeleOp OpModes**

- Compose:
  - `DriveIO` → `MecanumDrive`
  - `Localizer`
  - `TeleOpConfig` + helpers from `common/control`
- No direct 4-motor writes in production TeleOp.

**Auto OpModes**

- Use RoadRunner (via `common/rr`) + `MecanumDrive` + `Localizer`.
- No “god classes” mixing hardware, RR internals, and control logic.

---

## 5. Placeholder Constants Policy

When Codex needs a numeric value and none is defined:

- Use a **reasonable assumption** and mark:

```java
public static final double WHEEL_RADIUS_MM = 48.0; // TODO: measure actual
public static final double WHEEL_BASE_MM = 330.0;  // TODO: measure
public static final double TRACK_WIDTH_MM = 330.0; // TODO: measure

public static final double TICKS_PER_REV = 537.6;  // TODO: confirm motor
public static final double GEAR_RATIO = 1.0;       // TODO: confirm

public static final double MAX_WHEEL_SPEED_MM_PER_S = 1500.0; // TODO: tune
public static final double TELEOP_MAX_VEL_MM_PER_S = 1200.0;  // TODO: tune
public static final double TELEOP_MAX_ANG_VEL_RAD_PER_S = 4.0; // TODO: tune
```

Codex must:

- Never hide such values as unexplained magic numbers.
- Make it obvious where humans must calibrate.

---

## 6. Anti-Patterns (Never Do This)

Codex must NOT:

- Import FTC SDK types into `common/**` (except explicitly designated adapters/localizers).
- Recreate monolithic classes that:
  - Configure motors,
  - Track pose,
  - Follow trajectories,
  - Handle gamepads,
  - All in one.
- Change units away from mm/mm/s/rad/rad/s.
- Break the wheel order.
- Implement TeleOp logic inside `DriveIO`, `MecanumDrive`, `Localizer`, or `common/rr`.

If such patterns are found, Codex should refactor toward this spec.
