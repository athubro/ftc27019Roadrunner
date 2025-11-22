package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Turret subsystem - non-blocking tracking, manual control works correctly.
 * Public API kept the same as before.
 */
public final class Turret {
    public static class Params {
        public static final double PID_INTERVAL = 0.1;

        public double kP = 0.0001;
        public double kI = 0.0005;
        public double kD = 0.00001;

        public double toleranceRPM = 50.0;

        public static final double TICKS_PER_REV = 28.0;

        public static final int TARGET_TAG_ID = 20;
        public static final double TOLERANCE_DEG = 1.0;
        public static final double BASE_TURRET_POWER = 0.15;
        public static final double MIN_TURRET_POWER = 0.05;
        public static final double KP_TURRET = 0.02;
        public static final double SEARCH_POWER = 0.08;
        public static final long TRACK_SLEEP_MS = 20L;
    }

    public static Params PARAMS = new Params();

    // Hardware
    private final CRServo turret;          // continuous servo for yaw control
    private final Servo turretAngle;       // regular servo controlling turret angle
    private final Limelight3A limelight;
    private final DcMotor leftMotor;       // shooter left
    private final DcMotor rightMotor;      // shooter right
    private final FtcDashboard dashboard;

    // External telemetry (supplied by caller)
    private final Telemetry telemetry;

    // State (shooter PID)
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime pidTimer = new ElapsedTime();

    private double lastTime = 0.0;
    private int lastPositionLeft = 0;
    private int lastPositionRight = 0;

    private double integralLeft = 0.0;
    private double lastErrorLeft = 0.0;
    private double outputPowerLeft = 0.0;
    private double currentRPMLeft = 0.0;

    private double integralRight = 0.0;
    private double lastErrorRight = 0.0;
    private double outputPowerRight = 0.0;
    private double currentRPMRight = 0.0;

    // Turret tracking & control state
    public boolean trackingMode = false;   // whether limelight auto-tracks
    private double lastDirection = 1.0;    // last search direction when tag lost

    // Turret angle state (for Servo position)
    private double turretAnglePos = 0.5;   // start at middle position
    private static final double TURRET_ANGLE_STEP = 0.009;

    // External inputs (set by caller)
    private boolean shootingEnabled = false;
    private double manualTurretInput = 0.0; // store joystick value; update() applies it
    private int turretAngleCommand = 0;     // -1 left, 0 stop, 1 right (previously D-PAD)
    private double targetRPM = 4500.0;      // can be changed via setter

    // Constructor
    public Turret(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        turret = hardwareMap.get(CRServo.class, "turret");
        turretAngle = hardwareMap.get(Servo.class, "TurretAngle");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        leftMotor = hardwareMap.get(DcMotor.class, "left motor");
        rightMotor = hardwareMap.get(DcMotor.class, "right motor");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        dashboard = FtcDashboard.getInstance();

        // start limelight polling immediately
        limelight.setPollRateHz(100);
        limelight.start();

        // initialize timers / positions
        lastPositionLeft = leftMotor.getCurrentPosition();
        lastPositionRight = rightMotor.getCurrentPosition();
        lastTime = timer.seconds();
        pidTimer.reset();
    }

    // External setters (API preserved)
    public void setShootingEnabled(boolean enabled) {
        this.shootingEnabled = enabled;
    }

    // store manual turret input; update() will apply it when not tracking
    public void setManualTurretPower(double inputPower) {
        this.manualTurretInput = inputPower;
    }

    /**
     * turretAngleCommand: -1 left, 0 stop, +1 right
     */
    public void setTurretAngleCommand(int cmd) {
        this.turretAngleCommand = cmd;
    }

    public void setTrackingMode(boolean on) {
        this.trackingMode = on;
    }

    public void setTargetRPM(double rpm) {
        this.targetRPM = rpm;
    }

    public double getCurrentRPMLeft() { return currentRPMLeft; }
    public double getCurrentRPMRight() { return currentRPMRight; }
    public double getTargetRPM() { return targetRPM; }

    // -------------------------
    // Main update (call once per loop)
    // -------------------------
    public void update() {
        // update shooter PID (non-blocking)
        pidUpdate();

        // update turret yaw (non-blocking; either tracking or manual)
        updateTurretControl();

        // update turret angle servo (non-blocking)
        updateTurretAngle();

        // telemetry
        sendTelemetry();
        // no sleeps here — caller loop remains responsive
    }

    private void pidUpdate() {
        if (pidTimer.seconds() < Params.PID_INTERVAL) return;
        pidTimer.reset();

        int currentPositionL = leftMotor.getCurrentPosition();
        int currentPositionR = rightMotor.getCurrentPosition();
        double currentTime = timer.seconds();
        double deltaTime = currentTime - lastTime;
        if (deltaTime <= 0.0) deltaTime = 1e-6;

        // Left
        int deltaTicksLeft = currentPositionL - lastPositionLeft;
        double revsPerSecLeft = (deltaTicksLeft / Params.TICKS_PER_REV) / deltaTime;
        currentRPMLeft = revsPerSecLeft * 60.0;
        double errorLeft = targetRPM - currentRPMLeft;
        if (Math.abs(errorLeft) < PARAMS.toleranceRPM/2) {
            errorLeft = 0;
        }
        integralLeft += errorLeft * deltaTime;
        double derivativeLeft = (errorLeft - lastErrorLeft) / deltaTime;
        double pidOutputLeft = (PARAMS.kP * errorLeft) + (PARAMS.kI * integralLeft) + (PARAMS.kD * derivativeLeft);
        pidOutputLeft = clamper(pidOutputLeft, 0.0, 1.0);
        lastErrorLeft = errorLeft;
        lastPositionLeft = currentPositionL;

        // Right
        int deltaTicksRight = currentPositionR - lastPositionRight;
        double revsPerSecRight = (deltaTicksRight / Params.TICKS_PER_REV) / deltaTime;
        currentRPMRight = revsPerSecRight * 60.0;
        double errorRight = targetRPM - currentRPMRight;
        if (Math.abs(errorRight) < PARAMS.toleranceRPM/2) {
            errorRight = 0;
        }
        integralRight += errorRight * deltaTime;
        double derivativeRight = (errorRight - lastErrorRight) / deltaTime;
        double pidOutputRight = (PARAMS.kP * errorRight) + (PARAMS.kI * integralRight) + (PARAMS.kD * derivativeRight);
        pidOutputRight = clamper(pidOutputRight, 0.0, 1.0);
        lastErrorRight = errorRight;
        lastPositionRight = currentPositionR;

        lastTime = currentTime;

        // Apply shooter motor power if shooting is enabled
        if (shootingEnabled) {
            leftMotor.setPower(pidOutputLeft);
            rightMotor.setPower(pidOutputRight);
            outputPowerLeft = pidOutputLeft;
            outputPowerRight = pidOutputRight;
        } else {
            leftMotor.setPower(0.0);
            rightMotor.setPower(0.0);
            integralLeft = 0.0;
            integralRight = 0.0;
            outputPowerLeft = 0.0;
            outputPowerRight = 0.0;
        }
    }

    // Non-blocking turret yaw control: tracking or manual
    private void updateTurretControl() {
        double turretPower = 0.0;
        boolean tagFound = false;
        double errorAngleDeg = 0.0;

        if (trackingMode) {
            // read limelight result (non-blocking)
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                for (LLResultTypes.FiducialResult fid : result.getFiducialResults()) {
                    if (fid.getFiducialId() == Params.TARGET_TAG_ID) {
                        tagFound = true;
                        errorAngleDeg = fid.getTargetXDegrees();
                        break;
                    }
                }
            }

            if (tagFound) {
                if (Math.abs(errorAngleDeg) > Params.TOLERANCE_DEG) {
                    double proportionalPower = Params.KP_TURRET * Math.abs(errorAngleDeg);
                    proportionalPower = clamper(proportionalPower, Params.MIN_TURRET_POWER, Params.BASE_TURRET_POWER);
                    turretPower = (errorAngleDeg > 0) ? -proportionalPower : proportionalPower;
                    lastDirection = (errorAngleDeg > 0) ? -1.0 : 1.0;
                } else {
                    turretPower = 0.0;
                }
            } else {
                // simple search motion (non-blocking)
                turretPower = -lastDirection * Params.SEARCH_POWER;
            }
        } else {
            // manual control — direct feedthrough of the stored joystick value
            turretPower = manualTurretInput;
        }

        // finally apply power to turret (CRServo)
        turret.setPower(turretPower);

        telemetryData.tagFound = tagFound;
        telemetryData.errorAngleDeg = errorAngleDeg;
        telemetryData.turretPower = turretPower;
    }

    private void updateTurretAngle() {
        // turretAngleCommand changes the stored position incrementally
        if (turretAngleCommand > 0) {
            turretAnglePos += TURRET_ANGLE_STEP;
        } else if (turretAngleCommand < 0) {
            turretAnglePos -= TURRET_ANGLE_STEP;
        }

        turretAnglePos = clamper(turretAnglePos, 0.0, 1.0);
        turretAngle.setPosition(turretAnglePos);

        telemetryData.turretAnglePower = turretAnglePos;
    }

    private void sendTelemetry() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Left RPM", currentRPMLeft);
        packet.put("Right RPM", currentRPMRight);
        packet.put("Target RPM", targetRPM);
        dashboard.sendTelemetryPacket(packet);

        telemetry.addLine("=== SHOOTER PID ===");
        telemetry.addData("Left Motor RPM", "%.1f", currentRPMLeft);
        telemetry.addData("Right Motor RPM", "%.1f", currentRPMRight);
        telemetry.addData("Left Motor power", "%.2f", leftMotor.getPower());
        telemetry.addData("Right Motor power", "%.2f", rightMotor.getPower());
        telemetry.addData("Left ticks", leftMotor.getCurrentPosition());
        telemetry.addData("Right ticks", rightMotor.getCurrentPosition());
        telemetry.addData("Target RPM", "%.1f", targetRPM);

        telemetry.addLine("=== TURRET TRACKING ===");
        telemetry.addData("Tracking Mode", trackingMode ? "ON" : "OFF");
        telemetry.addData("Tag Visible", telemetryData.tagFound);
        telemetry.addData("Error Angle (deg)", "%.2f", telemetryData.errorAngleDeg);
        telemetry.addData("Turret Power", "%.3f", telemetryData.turretPower);

        telemetry.addLine("=== TURRET ANGLE ===");
        telemetry.addData("Turret Angle Position", "%.3f", telemetryData.turretAnglePower);

        telemetry.update();
    }

    // -------------------------
    // Helpers
    // -------------------------
    private static double clamper(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static class TelemetryData {
        boolean tagFound = false;
        double errorAngleDeg = 0.0;
        double turretPower = 0.0;
        double turretAnglePower = 0.0;
    }
    private final TelemetryData telemetryData = new TelemetryData();
}
