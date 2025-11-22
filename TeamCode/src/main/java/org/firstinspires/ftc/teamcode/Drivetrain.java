package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Drivetrain {

    // Servos
    private final Servo frontKick, middleKick, backKick;

    // Timer for non-blocking control
    private final ElapsedTime timer = new ElapsedTime();
    private final double kickerDuration = 300; // milliseconds

    // Kicker state
    private boolean frontActive = false;
    private boolean middleActive = false;
    private boolean backActive = false;

    public Drivetrain(HardwareMap hardwareMap) {
        frontKick = hardwareMap.get(Servo.class, "frontKick");
        middleKick = hardwareMap.get(Servo.class, "middleKick");
        backKick = hardwareMap.get(Servo.class, "backKick");

        // Initial positions
        frontKick.setPosition(0.0);
        middleKick.setPosition(0.0);
        backKick.setPosition(0.0);
    }

    /**
     * Call this every loop to update the kicker system
     * Pass the gamepad that controls the kickers
     */
    public void update(Gamepad gamepad) {
        // ---------- Front Kick ----------
        if (gamepad.a && !frontActive) {
            frontKick.setPosition(1.0);
            frontActive = true;
            timer.reset();
        }
        if (frontActive && timer.milliseconds() >= kickerDuration) {
            frontKick.setPosition(0.0);
            frontActive = false;
        }

        // ---------- Middle Kick ----------
        if (gamepad.b && !middleActive) {
            middleKick.setPosition(1.0);
            middleActive = true;
            timer.reset();
        }
        if (middleActive && timer.milliseconds() >= kickerDuration) {
            middleKick.setPosition(0.0);
            middleActive = false;
        }

        // ---------- Back Kick ----------
        if (gamepad.y && !backActive) {
            backKick.setPosition(1.0);
            backActive = true;
            timer.reset();
        }
        if (backActive && timer.milliseconds() >= kickerDuration) {
            backKick.setPosition(0.0);
            backActive = false;
        }
    }

    // Optional manual control
    public void fireFront() { frontKick.setPosition(1.0); frontActive = true; timer.reset(); }
    public void fireMiddle() { middleKick.setPosition(1.0); middleActive = true; timer.reset(); }
    public void fireBack() { backKick.setPosition(1.0); backActive = true; timer.reset(); }

    public void resetAll() {
        frontKick.setPosition(0.0);
        middleKick.setPosition(0.0);
        backKick.setPosition(0.0);
        frontActive = middleActive = backActive = false;
    }
}

