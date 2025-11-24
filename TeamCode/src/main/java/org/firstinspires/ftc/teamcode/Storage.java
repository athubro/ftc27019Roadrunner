package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Storage {

    private Servo frontKick, middleKick, backKick;

    private ColorSensor frontColor, middleColor, backColor;
    private DistanceSensor frontDis, middleDis, backDis;
    private NormalizedRGBA normalizedColors;
    private ElapsedTime timer = new ElapsedTime();
    public String[] colorArray = {"N", "N", "N"};
    public String[] ballArray = {"N", "N", "N"};
    public String frontBall = "N";
    public String middleBall = "N";
    public String backBall = "N";

    public Storage(HardwareMap hardwareMap) {
        frontKick = hardwareMap.get(Servo.class, "frontKick");
        middleKick = hardwareMap.get(Servo.class, "middleKick");
        backKick = hardwareMap.get(Servo.class, "backKick");
        middleKick.setDirection(Servo.Direction.REVERSE);

        frontKick.setPosition(0);
        middleKick.setPosition(0);
        backKick.setPosition(0);

        frontColor = hardwareMap.get(ColorSensor.class, "frontColor");
        middleColor = hardwareMap.get(ColorSensor.class, "middleColor");
        backColor = hardwareMap.get(ColorSensor.class, "backColor");

        frontDis = hardwareMap.get(DistanceSensor.class, "frontColor");
        middleDis = hardwareMap.get(DistanceSensor.class, "middleColor");
        backDis = hardwareMap.get(DistanceSensor.class, "backColor");

        ((NormalizedColorSensor) frontColor).setGain(82);
        ((NormalizedColorSensor) middleColor).setGain(82);
        ((NormalizedColorSensor) backColor).setGain(82);
    }

    // Update the color in each slot
    public void update() {
        colorArray[0] = detectColor(frontColor, frontDis, "front");
        colorArray[1] = detectColor(middleColor, middleDis, "middle");
        colorArray[2] = detectColor(backColor, backDis, "back");
        ballArray[0] = detectBall(frontDis,  "front");
        //--------------continue work herre------------------------------------------------------------------------
    }

    // Detect color
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
    private String detectColor(ColorSensor colorSensor, DistanceSensor distanceSensor, String slot) {
        normalizedColors = ((NormalizedColorSensor) colorSensor).getNormalizedColors();
        float R = normalizedColors.red;
        float G = normalizedColors.green;
        float B = normalizedColors.blue;

        double DisCheck = 3.7, greenRatio = 4.22;
        if (slot.equals("back")) { DisCheck = 7; greenRatio = 3; }
        if (slot.equals("middle")) { DisCheck = 5; greenRatio = 2.7; }
        if (slot.equals("front")) { DisCheck = 3; }

        double distance = distanceSensor.getDistance(DistanceUnit.CM);

        if (G > (1.1 * R) - 0.03 && G < (1.1 * R) + 0.03 && B > (2.02 * R) - 0.04 && B < (2.02 * R) + 0.04 && distance < DisCheck)
            return "P";
        else if (G > (greenRatio * R) - 0.05 && G < (greenRatio * R) + 0.05 && B > (2.35 * R) - 0.05 && B < (2.35 * R) + 0.05 && distance < DisCheck)
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

    private void loadBall(String color) {
        update(); // Refresh slot colors
        // Check front -> middle -> back for first available slot
       if (color == "P") {


       }
        // After kicking, update slot colors again
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
