package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Turret + Mecanum + Kickers", group = "TeleOp")
public class SolarStorm27019TeleOP extends LinearOpMode {

    private Turret turretSystem;
    private MecanumDrive drive;
    private Storage kickers;  // intake-free storage class
    private Pose2d pose = new Pose2d(0, 0, 0);

    @Override
    public void runOpMode() {

        // Initialize all systems
        turretSystem = new Turret(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap, pose);
        telemetry.addLine("new build");
        telemetry.update();
        kickers = new Storage(hardwareMap);  // intake-free kicker class

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
            turretSystem.setShootingEnabled(gamepad1.right_trigger > 0.1);

            if (gamepad1.dpad_up && !dpadUpLast)
                turretSystem.setTargetRPM(turretSystem.getTargetRPM() + 25.0);
            if (gamepad1.dpad_down && !dpadDownLast)
                turretSystem.setTargetRPM(Math.max(0, turretSystem.getTargetRPM() - 25.0));

            dpadUpLast = gamepad1.dpad_up;
            dpadDownLast = gamepad1.dpad_down;

            if (gamepad1.a && !aPressedLast)
                turretSystem.setTrackingMode(!turretSystem.trackingMode);

            aPressedLast = gamepad1.a;

            turretSystem.setManualTurretPower(gamepad1.right_stick_x);

            if (gamepad1.dpad_right)
                turretSystem.setTurretAngleCommand(1);
            else if (gamepad1.dpad_left)
                turretSystem.setTurretAngleCommand(-1);
            else
                turretSystem.setTurretAngleCommand(0);

            turretSystem.update();

            // =========================
            // Mecanum Drive Controls (Gamepad2)
            // =========================
            Vector2d translation = new Vector2d(-gamepad2.left_stick_y, -gamepad2.left_stick_x);
            double rotation = -gamepad2.right_stick_x;
            drive.setDrivePowers(new PoseVelocity2d(translation, rotation));

            // =========================
            // Kicker / Storage Updates
            // =========================
            kickers.update();  // senses colors and updates slot states
            //
            //
            //
            if(gamepad2.right_trigger > 0.1) kickers.setIntakePower(gamepad2.right_trigger);
            else if (gamepad2.left_trigger > 0.1)kickers.setIntakePower(-gamepad2.left_trigger);
            else kickers.setIntakePower(0);


            if (gamepad2.right_bumper) kickers.openGate();
            if (gamepad2.left_bumper) kickers.closeGate();
            // Automatic loading into nearest empty slot
            if (gamepad1.x) kickers.loadGreen();
            if (gamepad1.y) kickers.loadPurple();
            if (gamepad2.b) kickers.resetKick();


            // Manual kicking using D-pad
            if (gamepad2.dpad_up) kickers.kickBack();
            if (gamepad2.dpad_left) kickers.kickMiddle();
            if (gamepad2.dpad_down) kickers.kickFront();

            // Optional telemetry for debugging
            telemetry.addData("Front Slot", kickers.ballArray[0]);
            telemetry.addData("Middle Slot", kickers.ballArray[1]);
            telemetry.addData("Back Slot", kickers.ballArray[2]);
            telemetry.update();
        }
    }
}
