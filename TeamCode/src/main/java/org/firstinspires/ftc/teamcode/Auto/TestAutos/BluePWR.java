package org.firstinspires.ftc.teamcode.Auto.TestAutos;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.RingDetector;
import org.firstinspires.ftc.teamcode.MainRobot;


@Autonomous(name="BluePWR")
public class BluePWR extends LinearOpMode {

    MainRobot robot = new MainRobot();

    @Override
    public void runOpMode() throws InterruptedException {

        RingDetector detector = new RingDetector(this, false);
        //RingDetectorJHop detectorJHop = new RingDetectorJHop(this,false);


        robot.init(hardwareMap);

        robot.topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        robot.gyro.resetZAxisIntegrator();

        int rings = detector.getDecision();
        //int jHopRings = detectorJHop.getDecision();

        //---------------- CASE ZERO RINGS ----------------
        if (rings == 0){

            robot.gyroDrive(MainRobot.DRIVE_SPEED,-99,-99,-99,-99,0,-1,-1,-1,this);

            robot.gyroTurn(robot.TURN_SPEED,83,this);

            robot.wobbleArm.setPower(0.40);
            sleep(75);

            robot.wobbleClaw.setPower(0.5);
            sleep(1600);

            robot.wobbleArm.setPower(0);
            robot.wobbleClaw.setPower(0);

            //SHOOTING & PARKING
            robot.encoderDrive(MainRobot.DRIVE_SPEED,30,0,0,30,this); //STRAFES DIAGONALLY RIGHT
            robot.gyroDrive(0.6,0,0,0,0,0,50,60,-1,this); //CONFIRMS SHOOTING POSITION
            robot.gyroTurn(robot.TURN_SPEED,160,this); //TURNS TO PWR SHOT 3

            robot.jHopper2.setPower(-0.8);
            sleep(2000);
            /*while (detectorJHop.getDecision() > 0){
                robot.JHopFlap.setPosition(0.5);
                robot.jHopper1.setPower(-1);
                robot.foamWheel.setPower(1);
            }*/
            robot.JHopFlap.setPosition(0.5); //FLAP OPENS
            robot.jHopper1.setPower(-1);
            robot.foamWheel.setPower(1);
            sleep(1000);

            robot.gyroTurn(robot.TURN_SPEED,165,this); //TURNS TO PWR SHOT 2
            robot.jHopper1.setPower(-1);
            robot.foamWheel.setPower(1);
            sleep(1000);

            robot.gyroTurn(robot.TURN_SPEED,170,this); //TURNS TO PWR SHOT 1
            robot.jHopper1.setPower(-1);
            robot.foamWheel.setPower(1);
            sleep(1500);

            robot.jHopper1.setPower(0);
            robot.jHopper2.setPower(0);
            robot.JHopFlap.setPosition(0.1);

            robot.gyroTurn(robot.TURN_SPEED,180,this); //PERPENDICULAR TO LEFT WALL

            robot.gyroDrive(MainRobot.DRIVE_SPEED,5,5,5,5,0,-1,-1,-1,this); //PARKING
        }

        //---------------- CASE ONE RING ----------------
        else if(rings == 1){

            robot.encoderDrive(MainRobot.DRIVE_SPEED,0,-50,-50,0,this);
            robot.gyroDrive(MainRobot.DRIVE_SPEED,-101,-101,-101,-101,0,-1,-1,-1,this);

            robot.gyroTurn(robot.TURN_SPEED,-82,this);

            robot.wobbleArm.setPower(0.40);
            sleep(75);

            robot.wobbleClaw.setPower(0.5);
            sleep(1600);

            robot.wobbleArm.setPower(0);
            robot.wobbleClaw.setPower(0);

            //SHOOTING & PARKING
            robot.encoderDrive(MainRobot.DRIVE_SPEED,0,-100,-100,0,this); //STRAFES DIAGONALLY BACK-LEFT
            robot.gyroDrive(0.8,-10,-10,-10,-10,0,-1,-1,-1,this);
            robot.gyroTurn(robot.TURN_SPEED,90,this);
            robot.gyroDrive(0.6,0,0,0,0,0,67,42,-1,this);

            robot.jHopper2.setPower(-0.95);
            sleep(2000);
            /*while (detectorJHop.getDecision() > 0){
                robot.JHopFlap.setPosition(0.5);
                robot.jHopper1.setPower(-1);
                robot.foamWheel.setPower(1);
            }*/
            robot.JHopFlap.setPosition(0.5);
            robot.jHopper1.setPower(-1);
            robot.foamWheel.setPower(1);
            sleep(7000);

            robot.jHopper1.setPower(0);
            robot.jHopper2.setPower(0);
            robot.JHopFlap.setPosition(0.1);

            robot.gyroDrive(MainRobot.DRIVE_SPEED,8,8,8,8,0,-1,-1,-1,this); //PARKING
        }

        //---------------- CASE FOUR RINGS ----------------
        else if(rings == 4) {

            robot.encoderDrive(MainRobot.DRIVE_SPEED, 0, -60, -60, 0, this);
            robot.gyroDrive(MainRobot.DRIVE_SPEED, -115, -115, -115, -115, 0, -1, -1, -1, this);

            robot.wobbleArm.setPower(0.40);
            sleep(90);

            robot.wobbleClaw.setPower(0.8);
            sleep(1200);

            robot.wobbleArm.setPower(0);
            robot.wobbleClaw.setPower(0);

            //SHOOTING AND PARKING
            robot.encoderDrive(MainRobot.DRIVE_SPEED, 0, 150, 150, 0, this); //STRAFES DIAGONALLY LEFT
            //robot.encoderDrive(MainRobot.DRIVE_SPEED,0,-60,-60,0,this); //STRAFES LEFT TO SHOOTING POSITION
            robot.gyroTurn(robot.TURN_SPEED, 174, this);
            robot.gyroDrive(0.6, 0, 0, 0, 0, 0, 67, 42, -1, this);

            robot.jHopper2.setPower(-0.95);
            sleep(2000);
            /*while (detectorJHop.getDecision() > 0){
                robot.JHopFlap.setPosition(0.5);
                robot.jHopper1.setPower(-1);
                robot.foamWheel.setPower(1);
            }*/
            robot.JHopFlap.setPosition(0.5);
            robot.jHopper1.setPower(-1);
            robot.foamWheel.setPower(1);
            sleep(7000);

            robot.jHopper1.setPower(0);
            robot.jHopper2.setPower(0);
            robot.JHopFlap.setPosition(0.1);

            robot.gyroDrive(MainRobot.DRIVE_SPEED, 8, 8, 8, 8, 0, -1, -1, -1, this); //PARKING
        }
    }
}
