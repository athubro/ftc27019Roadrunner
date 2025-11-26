package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Storage {
    public static class Params {
        public static int backDisCheck = 7;
        public static int frontDisCheck = 3;
        public static int middleDisCheck = 5;
        public static double backGreenRatio = 3.0;
        public static double middleGreenRatio = 2.7;
        public static double boundaryGreen = 0.05;
        public static double boundaryUPPurple = 0.03;
        public static double boundaryLOWPurple = 0.04;
        public static double blueRedRatioGreen = 2.35;
        public static double blueRedRatioPurple = 2.02;
        public static double greenRedRatioPurple = 1.1;

    }
    public static Storage.Params PARAMS = new Storage.Params();
    private Servo frontKick, middleKick, backKick, gate;
    private CRServo transferServo;

    private ColorSensor frontColorSensor, middleColorSensor, backColorSensor;
    private DistanceSensor frontDisSensor, middleDisSensor, backDisSensor;
    private NormalizedRGBA normalizedColors;
    private DcMotor activeIntake;

    private ElapsedTime timer = new ElapsedTime();
    public String[] ballArray = {"N", "N", "N"};
    public String frontBall = "N";
    public String middleBall = "N";
    public String backBall = "N";
    // 3-ball cases (only move the kicker at the target ball)
    private void kickFront3()  { gate.setPosition(1); sleep(67);frontKick.setPosition(1); }
    private void kickMiddle3() {gate.setPosition(0.85); sleep(67); middleKick.setPosition(1); }
    private void kickBack3()   {gate.setPosition(0.85); sleep(67); backKick.setPosition(1); }

    // 2-ball targeted kicks
    private void kickBack2Front()   { gate.setPosition(0.85); sleep(67); backKick.setPosition(1); sleep(67); middleKick.setPosition(1); }
    private void kickBack2Middle()  {gate.setPosition(0.85); sleep(67); backKick.setPosition(1); sleep(67); frontKick.setPosition(1); }
    private void kickMiddle2Front() { gate.setPosition(0.85); sleep(67); middleKick.setPosition(1); sleep(67); backKick.setPosition(1); }
    private void kickMiddle2Back()  { gate.setPosition(0.85); sleep(67); middleKick.setPosition(1); sleep(67); frontKick.setPosition(1); }
    private void kickFront2Back()   { gate.setPosition(1); sleep(67); frontKick.setPosition(1); sleep(67); middleKick.setPosition(1); }
    private void kickFront2Middle() { gate.setPosition(1); sleep(67);frontKick.setPosition(1); sleep(67); backKick.setPosition(1); }

    // 1-ball cases (have to actuate all downstream kickers)
    private void kickFront1()  {gate.setPosition(1); sleep(67); frontKick.setPosition(1); sleep(67); backKick.setPosition(1); sleep(67); middleKick.setPosition(1); }
    private void kickMiddle1() { gate.setPosition(0.85); sleep(67); middleKick.setPosition(1); sleep(67); backKick.setPosition(1); sleep(67); frontKick.setPosition(1); }
    private void kickBack1()   { gate.setPosition(0.85); sleep(67); backKick.setPosition(1); sleep(67); middleKick.setPosition(1); sleep(67); frontKick.setPosition(1); }

    public Storage(HardwareMap hardwareMap) {
        frontKick = hardwareMap.get(Servo.class, "frontKick");
        middleKick = hardwareMap.get(Servo.class, "middleKick");
        backKick = hardwareMap.get(Servo.class, "backKick");
        gate = hardwareMap.get(Servo.class, "gate");
        transferServo = hardwareMap.get(CRServo.class, "transferServo");
        activeIntake = hardwareMap.get(DcMotor.class, "activeIntake");
        // backKick : deepest slot
        // middleKick : 2nd deepest slot
        //frontKick : closest to intake slot
        middleKick.setDirection(Servo.Direction.REVERSE);

        frontKick.setPosition(0);
        middleKick.setPosition(0);
        backKick.setPosition(0);

        frontColorSensor = hardwareMap.get(ColorSensor.class, "frontColor");
        middleColorSensor = hardwareMap.get(ColorSensor.class, "middleColor");
        backColorSensor = hardwareMap.get(ColorSensor.class, "backColor");

        frontDisSensor = hardwareMap.get(DistanceSensor.class, "frontColor");
        middleDisSensor = hardwareMap.get(DistanceSensor.class, "middleColor");
        backDisSensor = hardwareMap.get(DistanceSensor.class, "backColor");

        ((NormalizedColorSensor) frontColorSensor).setGain(82);
        ((NormalizedColorSensor) middleColorSensor).setGain(82);
        ((NormalizedColorSensor) backColorSensor).setGain(82);
    }

    // Update the color in each slot
    public void update() {
        ballArray[0] = detectColor(frontColorSensor, frontDisSensor, "front");
        ballArray[1] = detectColor(middleColorSensor, middleDisSensor, "middle");
        ballArray[2] = detectColor(backColorSensor, backDisSensor, "back");

    }

    // not used
    /*
    private String detectBall (DistanceSensor distanceSensor, String slot) {
        String ball = "";
        double distance = distanceSensor.getDistance(DistanceUnit.CM);


        if (slot.equals("back")) {
            if (distance < 7) {
                ball = "Y";
            } else {
                ball = "N";
            }
        } else if (slot.equals("middle")) {
            if (distance < 5) {
                ball = "Y";
            } else {
                ball = "N";
            }

        } else if (slot.equals("front")) {
            if (distance < 3) {
                ball = "Y";
            } else {
                ball = "N";
            }

        }
        return ball;

    }

     */
    public void setIntakePower(double inputPower) {
        if (inputPower > 0.15) {
            gate.setPosition(0);
            activeIntake.setPower(inputPower);
            transferServo.setPower(1);
        } else if (inputPower < -0.15){
            activeIntake.setPower(inputPower);
        }
        else {
            gate.setPosition(1);
        }
    }
    public void openGate(){
        gate.setPosition(0);
    }
    public void closeGate(){
        gate.setPosition(1);
    }
    private String detectColor(ColorSensor colorSensor, DistanceSensor distanceSensor, String slot) {
        normalizedColors = ((NormalizedColorSensor) colorSensor).getNormalizedColors();
        float R = normalizedColors.red;
        float G = normalizedColors.green;
        float B = normalizedColors.blue;

        double DisCheck = 3.7, greenRatio = 4.22;
        if (slot.equals("back")) { DisCheck = PARAMS.backDisCheck; greenRatio = PARAMS.backGreenRatio; }
        if (slot.equals("middle")) { DisCheck = PARAMS.middleDisCheck; greenRatio = PARAMS.middleGreenRatio; }
        if (slot.equals("front")) { DisCheck = PARAMS.frontDisCheck; }

        double distance = distanceSensor.getDistance(DistanceUnit.CM);

        if (G > (PARAMS.greenRedRatioPurple * R) - PARAMS.boundaryUPPurple && G < (PARAMS.greenRedRatioPurple * R) + PARAMS.boundaryUPPurple && B > (PARAMS.blueRedRatioPurple * R) - PARAMS.boundaryLOWPurple && B < (PARAMS.blueRedRatioPurple * R) + PARAMS.boundaryLOWPurple && distance < DisCheck)
            return "P";
        else if (G > (greenRatio * R) - PARAMS.boundaryGreen && G < (greenRatio * R) + PARAMS.boundaryGreen && B > (PARAMS.blueRedRatioGreen * R) - PARAMS.boundaryGreen && B < (PARAMS.blueRedRatioGreen * R) + PARAMS.boundaryGreen && distance < DisCheck)
            return "G";
        else
            return "N";
    }

    // Kickers
    public void kickFront() { frontKick.setPosition(1); }
    public void kickMiddle() { middleKick.setPosition(1); }
    public void kickBack() { backKick.setPosition(1); }
    public void resetKick() {backKick.setPosition(0); sleep(67); middleKick.setPosition(0); sleep(67); frontKick.setPosition(0);  }
    private void sleep(int ms) { timer.reset(); while(timer.milliseconds() < ms) {} }

    // Automated loading for green ball
    public void loadGreen() { loadBall("G"); }


    // Automated loading for purple ball
    public void loadPurple() { loadBall("P"); }

    private void loadBall(String targetColor) { // "G" or "P"
        update(); // Refresh sensor data

        String f = ballArray[0]; // front
        String m = ballArray[1]; // middle
        String b = ballArray[2]; // back

        // Count how many balls are currently in storage
        int count = 0;
        if (!f.equals("N")) count++;
        if (!m.equals("N")) count++; //count is how many empty spaces
        if (!b.equals("N")) count++;

        // Find the FIRST (closest to intake) ball that matches the target color
        if (f.equals(targetColor)) {
            // Target ball is in FRONT
            if (count == 0) {
                kickFront3();
            } else if (count == 1) {
                if (m.equals("N")) kickFront2Back();    // front + back
                else               kickFront2Middle();  // front + middle
            } else if (count == 2) {
                kickFront1();
            }
        }
        else if (m.equals(targetColor)) {
            // Target ball is in MIDDLE
            if (count == 0) {
                kickMiddle3();
            } else if (count == 1) {
                if (f.equals("N")) kickMiddle2Back();   // middle + back
                else {
                    kickMiddle2Front();
                }    // middle + front
            } else if (count == 2) {
                kickMiddle1();
            }
        }
        else if (b.equals(targetColor)) {
            // Target ball is in BACK
            if (count == 0) {
                kickBack3();
            } else if (count == 1) {
                if (f.equals("N")) kickBack2Middle();   // back + middle
                else               kickBack2Front();    // back + front
            } else if (count == 2) {
                kickBack1();
            }
        }
        // If no matching ball found → do nothing

        // Always reset kickers and refresh state
        resetKick();
        update();
    }

    // Count balls in storage
    public int ballCount() {
        int count = 0;
        if (!frontBall.equals("N")) count++;
        if (!middleBall.equals("N")) count++;
        if (!backBall.equals("N")) count++;
        return count;
    }
}
