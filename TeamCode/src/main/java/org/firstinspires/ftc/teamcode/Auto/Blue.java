package org.firstinspires.ftc.teamcode.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.android.dx.command.Main;
import org.firstinspires.ftc.teamcode.MainRobot;


@Autonomous(name="Blue")
public class Blue extends LinearOpMode {

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
        detector.setTelemShow(false);
        //int jHopRings = detectorJHop.getDecision();

        //---------------- CASE ZERO RINGS ----------------
        if (rings == 0){

            robot.gyroDrive(MainRobot.DRIVE_SPEED,-99,-99,-99,-99,0,0,0.2,0,0.2,this);
            robot.gyroTurn(MainRobot.TURN_SPEED,83,this);

            robot.wobbleArm.setPower(0.5);
            sleep(45);

            robot.wobbleClaw.setPower(1);
            sleep(1200);

            robot.wobbleArm.setPower(0);
            robot.wobbleClaw.setPower(0);

            //SHOOTING & PARKING
            robot.encoderDrive(MainRobot.DRIVE_SPEED,73,0,0,73,this); //STRAFES DIAGONALLY RIGHT

            robot.jHopper2.setPower(-0.95);
            robot.gyroTurn(MainRobot.TURN_SPEED,175,this); //TURNS TO SHOOTING ANGLE
            sleep(800);
            robot.JHopFlap.setPosition(0.5);
            robot.jHopper1.setPower(-1);
            robot.foamWheel.setPower(1);
            robot.sleepV2(3.4, this);

            robot.jHopper1.setPower(0);
            robot.jHopper2.setPower(0);
            robot.foamWheel.setPower(0);
            robot.JHopFlap.setPosition(0.1);

            robot.encoderDrive(MainRobot.DRIVE_SPEED,25,25,25,25,this); //PARKING
            robot.gyroTurn(0.3,180,this); //PERPENDICULAR TO FRONT WALL
        }

        //---------------- CASE ONE RING ----------------
        else if(rings == 1){

            robot.encoderDrive(MainRobot.DRIVE_SPEED,0,-47,-47,0,this);
            robot.encoderDrive(MainRobot.DRIVE_SPEED,-105,-105,-105,-105,this);

            robot.gyroTurn(MainRobot.TURN_SPEED,-80,this);

            robot.wobbleArm.setPower(0.5);
            sleep(45);

            robot.wobbleClaw.setPower(1);
            sleep(1200);

            robot.wobbleArm.setPower(0);
            robot.wobbleClaw.setPower(0);

            //SHOOTING & PARKING
            robot.encoderDrive(MainRobot.DRIVE_SPEED,-127,0,0,-127,this); //STRAFES DIAGONALLY BACK-LEFT
            robot.gyroTurn(MainRobot.TURN_SPEED,189,this);

            robot.jHopper2.setPower(-0.95);
            sleep(2500);
            /*while (detectorJHop.getDecision() > 0){
                robot.JHopFlap.setPosition(0.5);
                robot.jHopper1.setPower(-1);
                robot.foamWheel.setPower(1);
            }*/
            robot.JHopFlap.setPosition(0.5);
            robot.jHopper1.setPower(-1);
            robot.foamWheel.setPower(1);
            robot.sleepV2(3.4, this);

            robot.jHopper1.setPower(0);
            robot.jHopper2.setPower(0);
            robot.foamWheel.setPower(0);
            robot.JHopFlap.setPosition(0.1);

            robot.encoderDrive(MainRobot.DRIVE_SPEED,25,25,25,25,this); //PARKING
            robot.gyroTurn(0.3,180,this); //PERPENDICULAR TO FRONT WALL
        }

        //---------------- CASE FOUR RINGS ----------------
        else if(rings == 4) {

            robot.encoderDrive(0.8, 0, -39, -39, 0, this);//DIAGONAL BACK-RIGHT
            robot.gyroTurn(MainRobot.TURN_SPEED,0,this);
            robot.encoderDrive(0.8,-120,-120,-120,-120,this);
            robot.gyroTurn(MainRobot.TURN_SPEED,0,this);
            //robot.rangeDrive(0.1,-1,11,-1,this);

            robot.wobbleArm.setPower(0.5);
            robot.sleepV2(0.045,this);

            robot.wobbleClaw.setPower(1);
            robot.sleepV2(1.2,this);

            robot.wobbleArm.setPower(0);

            //SHOOTING AND PARKING
            robot.encoderDrive(MainRobot.DRIVE_SPEED,0,132,132,0,this); //STRAFES DIAGONALLY LEFT
            robot.gyroTurn(0.2,0,this);
            robot.gyroTurn(MainRobot.TURN_SPEED, 170.9, this);

            //robot.rangeDrive(0.3,65.4,45.3,-1,this);

            robot.jHopper2.setPower(-1);
            sleep(2500);
            /*while (detectorJHop.getDecision() > 0){
                robot.JHopFlap.setPosition(0.5);
                robot.jHopper1.setPower(-1);
                robot.foamWheel.setPower(1);
            }*/
            robot.JHopFlap.setPosition(0.5);
            robot.jHopper1.setPower(-1);
            robot.foamWheel.setPower(1);
            robot.sleepV2(3.4, this);

            robot.jHopper1.setPower(0);
            robot.jHopper2.setPower(0);
            robot.foamWheel.setPower(0);
            robot.JHopFlap.setPosition(0.1);

            robot.wobbleClaw.setPower(0);
            robot.encoderDrive(MainRobot.DRIVE_SPEED, 25, 25, 25, 25, this);
            robot.gyroTurn(0.3,180,this); //PERPENDICULAR TO FRONT WALL
        }
    }
}
