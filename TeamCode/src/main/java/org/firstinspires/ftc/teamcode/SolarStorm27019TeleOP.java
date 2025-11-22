package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Turret + Mecanum + Kickers", group = "TeleOp")
public class SolarStorm27019TeleOP extends LinearOpMode {

    private Turret turretSystem;
    private MecanumDrive drive;
    private Drivetrain kickers;  // updated class
    private Pose2d pose = new Pose2d(0, 0, 0);

    @Override
    public void runOpMode() {

        // Initialize all systems
        turretSystem = new Turret(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap, pose);
        kickers = new Drivetrain(hardwareMap);  // initialize new kicker class

        telemetry.addLine("Turret + Drive + Kickers Ready");
        telemetry.update();
        waitForStart();

        // Button state trackers for toggles
        boolean aPressedLast = false;
        boolean dpadUpLast = false;
        boolean dpadDownLast = false;

        while (opModeIsActive()) {

            // =========================
            // Turret & Shooter Controls (Gamepad1)
            // =========================

            // Shooter enable: RT > 0.1
            turretSystem.setShootingEnabled(gamepad1.right_trigger > 0.1);

            // RPM up/down
            if (gamepad1.dpad_up && !dpadUpLast)
                turretSystem.setTargetRPM(turretSystem.getTargetRPM() + 25.0);

            if (gamepad1.dpad_down && !dpadDownLast)
                turretSystem.setTargetRPM(Math.max(0, turretSystem.getTargetRPM() - 25.0));

            dpadUpLast = gamepad1.dpad_up;
            dpadDownLast = gamepad1.dpad_down;

            // Toggle tracking mode
            if (gamepad1.a && !aPressedLast)
                turretSystem.setTrackingMode(!turretSystem.trackingMode);

            aPressedLast = gamepad1.a;

            // Manual turret (used ONLY when NOT tracking)
            turretSystem.setManualTurretPower(gamepad1.right_stick_x);

            // Angle step commands
            if (gamepad1.dpad_right)
                turretSystem.setTurretAngleCommand(1);
            else if (gamepad1.dpad_left)
                turretSystem.setTurretAngleCommand(-1);
            else
                turretSystem.setTurretAngleCommand(0);

            // Update turret ALWAYS (tracking OR manual BOTH allowed)
            turretSystem.update();


            // =========================
            // Mecanum Drive Controls (Gamepad2)
            // =========================
            Vector2d translation = new Vector2d(-gamepad2.left_stick_y, gamepad2.left_stick_x);
            double rotation = gamepad2.right_stick_x;
            drive.setDrivePowers(new PoseVelocity2d(translation, rotation));


            // =========================
            // Kicker Controls (Gamepad2)
            // =========================
            kickers.update(gamepad2);  // new non-blocking kicker update
        }
    }
}
