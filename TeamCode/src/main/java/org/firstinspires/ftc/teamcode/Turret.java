package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

@TeleOp(name = "Turret", group = "Autonomous")
public class Turret extends LinearOpMode {

    // Shooter Motors
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime pidTimer = new ElapsedTime();
    private static final double PID_INTERVAL = 0.03;

    private double kP = 0.01;
    private double kI = 0.000001;
    private double kD = 0.0000015;

    private double targetRPM = 4500.0;
    private double toleranceRPM = 15.0;

    private double integralLeft = 0.0;
    private double lastErrorLeft = 0.0;
    private int lastPositionLeft = 0;
    private double outputPowerLeft = 0.0;
    private double currentRPMLeft = 0.0;

    private double integralRight = 0.0;
    private double lastErrorRight = 0.0;
    private int lastPositionRight = 0;
    private double outputPowerRight = 0.0;
    private double currentRPMRight = 0.0;

    private boolean dpadUpLast = false;
    private boolean dpadDownLast = false;

    private static final double TICKS_PER_REV = 28.0;

    // Turret + Limelight
    private CRServo turret;
    private CRServo turretAngle; // stays CRServo, now controlled by D-pad
    private Limelight3A limelight;
    private FtcDashboard dashboard;
    private MultipleTelemetry multiTelemetry;

    private static final int TARGET_TAG_ID = 20;
    private static final double TOLERANCE_DEG = 1.0;
    private static final double BASE_TURRET_POWER = 0.15;
    private static final double MIN_TURRET_POWER = 0.05;
    private static final double KP_TURRET = 0.02;
    private static final double SEARCH_POWER = 0.08;
    private static final long TRACK_SLEEP_MS = 20L;

    private boolean trackingMode = false;
    private boolean aPressedLast = false;
    private double lastDirection = 1.0;

    private double lastTime = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        turret = hardwareMap.get(CRServo.class, "turret");
        turretAngle = hardwareMap.get(CRServo.class, "TurretAngle");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        leftMotor = hardwareMap.get(DcMotor.class, "left motor");
        rightMotor = hardwareMap.get(DcMotor.class, "right motor");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        dashboard = FtcDashboard.getInstance();
        multiTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry = multiTelemetry;

        limelight.setPollRateHz(100);
        limelight.start();

        lastPositionLeft = leftMotor.getCurrentPosition();
        lastPositionRight = rightMotor.getCurrentPosition();
        lastTime = timer.seconds();
        pidTimer.reset();

        telemetry.addLine("Turret + Shooter PID Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // Toggle tracking
            if (gamepad1.a && !aPressedLast) trackingMode = !trackingMode;
            aPressedLast = gamepad1.a;

            // Change shooter RPM
            if (gamepad1.dpad_up && !dpadUpLast) targetRPM += 25.0;
            if (gamepad1.dpad_down && !dpadDownLast) targetRPM = Math.max(0, targetRPM - 25.0);
            dpadUpLast = gamepad1.dpad_up;
            dpadDownLast = gamepad1.dpad_down;

            // Shooter PID update timing
            if (pidTimer.seconds() >= PID_INTERVAL) {
                pidTimer.reset();

                int currentPositionL = leftMotor.getCurrentPosition();
                int currentPositionR = rightMotor.getCurrentPosition();
                double currentTime = timer.seconds();
                double deltaTime = currentTime - lastTime;
                if (deltaTime <= 0.0) deltaTime = 0.001;

                int deltaTicksLeft = currentPositionL - lastPositionLeft;
                double revsPerSecLeft = (deltaTicksLeft / TICKS_PER_REV) / deltaTime;
                currentRPMLeft = revsPerSecLeft * 60.0;
                double errorLeft = targetRPM - currentRPMLeft;
                if (Math.abs(errorLeft) > toleranceRPM) integralLeft += errorLeft * deltaTime;
                else integralLeft *= 0.9;
                double derivativeLeft = (errorLeft - lastErrorLeft) / deltaTime;
                double pidOutputLeft = (kP * errorLeft) + (kI * integralLeft) + (kD * derivativeLeft);
                pidOutputLeft = Math.max(0.0, Math.min(1.0, pidOutputLeft));
                lastErrorLeft = errorLeft;
                lastPositionLeft = currentPositionL;

                int deltaTicksRight = currentPositionR - lastPositionRight;
                double revsPerSecRight = (deltaTicksRight / TICKS_PER_REV) / deltaTime;
                currentRPMRight = revsPerSecRight * 60.0;
                double errorRight = targetRPM - currentRPMRight;
                if (Math.abs(errorRight) > toleranceRPM) integralRight += errorRight * deltaTime;
                else integralRight *= 0.9;
                double derivativeRight = (errorRight - lastErrorRight) / deltaTime;
                double pidOutputRight = (kP * errorRight) + (kI * integralRight) + (kD * derivativeRight);
                pidOutputRight = Math.max(0.0, Math.min(1.0, pidOutputRight));
                lastErrorRight = errorRight;
                lastPositionRight = currentPositionR;

                lastTime = currentTime;

                if (gamepad1.right_trigger > 0.1) {
                    leftMotor.setPower(pidOutputLeft);
                    rightMotor.setPower(pidOutputRight);
                    outputPowerLeft = pidOutputLeft;
                    outputPowerRight = pidOutputRight;
                } else {
                    leftMotor.setPower(0.0);
                    rightMotor.setPower(0.0);
                    integralLeft = 0.0;
                    integralRight = 0.0;
                }
            }

            // Limelight turret tracking
            double turretPower = 0.0;
            boolean tagFound = false;
            double errorAngleDeg = 0.0;

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                for (LLResultTypes.FiducialResult fid : result.getFiducialResults()) {
                    if (fid.getFiducialId() == TARGET_TAG_ID) {
                        tagFound = true;
                        errorAngleDeg = fid.getTargetXDegrees();
                        break;
                    }
                }
            }

            if (trackingMode) {
                if (tagFound) {
                    if (Math.abs(errorAngleDeg) > TOLERANCE_DEG) {
                        double proportionalPower = KP_TURRET * Math.abs(errorAngleDeg);
                        proportionalPower = Math.max(MIN_TURRET_POWER, Math.min(BASE_TURRET_POWER, proportionalPower));
                        turretPower = (errorAngleDeg > 0) ? -proportionalPower : proportionalPower;
                        lastDirection = (errorAngleDeg > 0) ? -1.0 : 1.0;
                    } else turretPower = 0.0;
                } else turretPower = -lastDirection * SEARCH_POWER;
            } else turretPower = gamepad1.right_stick_x;

            turret.setPower(turretPower);

            // *** UPDATED TURRET ANGLE CONTROL (D-PAD LEFT / RIGHT) ***
            if (gamepad1.dpad_right) {
                turretAngle.setPower(0.4);
            } else if (gamepad1.dpad_left) {
                turretAngle.setPower(-0.4);
            } else {
                turretAngle.setPower(0.0);
            }

            // Telemetry
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Left RPM", currentRPMLeft);
            packet.put("Right RPM", currentRPMRight);
            packet.put("Target RPM", targetRPM);
            dashboard.sendTelemetryPacket(packet);

            telemetry.addLine("=== SHOOTER PID ===");
            telemetry.addData("Left Motor RPM", "%.1f", currentRPMLeft);
            telemetry.addData("Right Motor RPM", "%.1f", currentRPMRight);
            telemetry.addData("Target RPM", "%.1f", targetRPM);

            telemetry.addLine("=== TURRET TRACKING ===");
            telemetry.addData("Tracking Mode", trackingMode ? "ON" : "OFF");
            telemetry.addData("Tag Visible", tagFound);
            telemetry.addData("Error Angle (deg)", "%.2f", errorAngleDeg);
            telemetry.addData("Turret Power", "%.3f", turretPower);

            telemetry.addLine("=== TURRET ANGLE ===");
            telemetry.addData("Turret Angle Power", "%.3f", turretAngle.getPower());

            telemetry.update();

            if (trackingMode) sleep(TRACK_SLEEP_MS);
        }

        limelight.close();
    }
}
