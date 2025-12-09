package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SSMyRobot  {

    private Turret turretSystem;
    private MecanumDrive drive;
    //private Storage kickers;  // intake-free storage class
    private StorageWLoader kickers;
    private Pose2d pose ;
    //private boolean waitingForConfig = true;
    private double speedRatio = 0.4;
    public ElapsedTime generalTimer = new ElapsedTime();
    private HardwareMap myHardwareMap;

    public SSMyRobot (HardwareMap hardwareMap, MecanumDrive myDrive, StorageWLoader storage, Turret turret, Pose2d newPos) {
        kickers=storage;
        drive = myDrive;
        turretSystem=turret;
        myHardwareMap=hardwareMap;
        pose=newPos;

    }

/*
    public class myFunction implements Action {
        public boolean run(@NonNull TelemetryPacket pack) {
            if () return true;
            else return false;
        }
    }

    public Action MyFunction() {
        return new myFunction();
    }


    public class calcShotVariables implements Action {
        public boolean run(@NonNull TelemetryPacket pack) {
            double startingTime= generalTimer.seconds();
            double timeOut= 5; // 5 sec to time out;


            turretSystem.updateTurretControl();
            turretSystem.measureDis();
            turretSystem.calcTargetAngleSpeed();
            if (turretSystem.tagFound) return true;
            else return false;
        }
    }

    public Action CalcShotVariables() {
        return new calcShotVariables();
    }


    public class shooterSpinUp implements Action {
        public boolean run(@NonNull TelemetryPacket pack) {
            if () return true;
            else return false;
        }
    }

    public Action ShooterSpinUp() {
        return new shooterSpinUp();
    }
    /
 */

    public class LoadGreenAction implements Action {
        public boolean run(@NonNull TelemetryPacket pack){
            double startingTime= generalTimer.seconds();
            double timeOut= 5; // 5 sec to time out;
            boolean finished =false;
            boolean shotFlag=false;
            boolean storageFlag=false;

            turretSystem.update();
            kickers.update();  // senses colors and updates slot states
            kickers.loadingUpdate();

            shotDetectReset();
            kickers.loadGreen();
            while (!finished && generalTimer.seconds()<startingTime+timeOut){
                turretSystem.update();
                kickers.update();  // senses colors and updates slot states
                kickers.loadingUpdate();

                shotFlag=shotFlag || turretSystem.shotDetected;
                if (shotFlag){
                    storageFlag=storageFlag || kickers.flag;
                }
                finished= shotFlag && storageFlag;
                shotDetectReset();

            }

            if (finished) return true;
            else return false;

        }


    }

    public Action loadGreenAction() {
        return new LoadGreenAction();

    }





    public class LoadPurpleAction implements Action {
        public boolean run(@NonNull TelemetryPacket pack){
            double startingTime= generalTimer.seconds();
            double timeOut= 5; // 5 sec to time out;
            boolean finished =false;
            boolean shotFlag=false;
            boolean storageFlag=false;

            turretSystem.update();
            kickers.update();  // senses colors and updates slot states
            kickers.loadingUpdate();

            shotDetectReset();
            kickers.loadPurple();
            while (!finished && generalTimer.seconds()<startingTime+timeOut){
                turretSystem.update();
                kickers.update();  // senses colors and updates slot states
                kickers.loadingUpdate();

                shotFlag=shotFlag || turretSystem.shotDetected;
                if (shotFlag){
                    storageFlag=storageFlag || kickers.flag;
                }
                finished= shotFlag && storageFlag;
                shotDetectReset();

            }

            if (finished) return true;
            else return false;

        }


    }

    public Action loadPurpleAction() {
        return new LoadPurpleAction();

    }


    public void shotDetectReset() {
        if (kickers.kickUp && turretSystem.shotDetected) {
            kickers.resetKick();
            turretSystem.shotDetected = false;
        }
    }
    /*
    public void rapidFire(){
        int count = kickers.ballCount();
        kickers.update();
        for (int i = 0; i < 3; i++) {
            if (kickers.ballArray[i] == "P") {
                kickers.loadPurple();
                while (!kickers.flag && turretSystem.shotDetected) {
                    kickers.update();
                    turretSystem.update();

                }
                shotDetectReset();
                sleep(400);

            } else if (kickers.ballArray[i] == "G") {
                kickers.loadGreen();
                kickers.loadPurple();
                while (!kickers.flag && turretSystem.shotDetected) {
                    kickers.update();
                    turretSystem.update();

                }
                shotDetectReset();
                sleep(400);

            } else {

            }
        }
    }
    */


}
