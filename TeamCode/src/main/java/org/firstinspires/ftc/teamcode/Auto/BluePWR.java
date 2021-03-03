package org.firstinspires.ftc.teamcode.Auto;


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
            robot.encoderDrive(0.3,6,6,6,6,this);

            //POWER-SHOT ONE
            robot.jHopper2.setPower(-0.78);
            robot.gyroTurn(MainRobot.TURN_SPEED,-28.6,this); //TURNS TO PWR SHOT RIGHT
            sleep(1700);

            robot.JHopFlap.setPosition(0.5); //FLAP OPENS
            robot.jHopper1.setPower(-1);
            robot.foamWheel.setPower(1);
            sleep(1000);

            robot.JHopFlap.setPosition(0.1); //FLAP CLOSES
            robot.jHopper1.setPower(0);
            robot.foamWheel.setPower(0);

            //POWER-SHOT TWO
            robot.gyroTurn(MainRobot.TURN_SPEED,-21,this); //TURNS TO PWR SHOT MIDDLE
            robot.JHopFlap.setPosition(0.5); //FLAP OPENS
            robot.jHopper1.setPower(-1);
            robot.foamWheel.setPower(1);
            sleep(1000);

            robot.JHopFlap.setPosition(0.1); //FLAP CLOSES
            robot.jHopper1.setPower(0);
            robot.foamWheel.setPower(0);

            //POWER-SHOT THREE
            robot.gyroTurn(MainRobot.TURN_SPEED,-19.8,this); //TURNS TO PWR SHOT LEFT
            robot.JHopFlap.setPosition(0.5); //FLAP OPENS
            robot.jHopper1.setPower(-1);
            robot.foamWheel.setPower(1);
            sleep(1000);

            robot.JHopFlap.setPosition(0.1); //FLAP CLOSES
            robot.jHopper1.setPower(0);
            robot.foamWheel.setPower(0);
            robot.jHopper2.setPower(0);

            //WOBBLE & PARKING
            robot.gyroTurn(MainRobot.TURN_SPEED,0,this);
            robot.encoderDrive(0.9,45,45,45,45,this); //STRAFES FORWARD
            robot.gyroTurn(MainRobot.TURN_SPEED,0,this);

            robot.wobbleArm.setPower(0.2);
            robot.encoderDrive(0.9,0,18,18,0,this); //DIAGONALLY FRONT-LEFT
            robot.gyroTurn(MainRobot.TURN_SPEED,0,this);
            robot.encoderDrive(MainRobot.DRIVE_SPEED,30,30,30,30,this); //STRAFES FORWARD TO BOX B
            robot.gyroTurn(MainRobot.TURN_SPEED,-140,this);
            robot.wobbleArm.setPower(0);

            robot.wobbleClaw.setPower(1);
            sleep(600);
            robot.gyroTurn(MainRobot.TURN_SPEED,0,this);
            robot.wobbleClaw.setPower(0);

            robot.encoderDrive(MainRobot.DRIVE_SPEED,-5,-5,-5,-5,this); //PARKING

            robot.gyroTurn(MainRobot.TURN_SPEED,0,this); //PARALLEL TO LEFT WALL

            //2ND WOBBLE & PARKING (IN PROGRESS)
            /*robot.encoderDrive(MainRobot.DRIVE_SPEED,0,-70,-70,0,this); //DIAGONALLY BACK-LEFT
            robot.encoderDrive(MainRobot.DRIVE_SPEED,-20,-20,-20,-20,this); //HITS BACK WALL
            robot.encoderDrive(MainRobot.DRIVE_SPEED,15,-15,-15,15,this); //STRAFES RIGHT
            robot.encoderDrive(MainRobot.DRIVE_SPEED,0,80,80,0,this); //DIAGONALLY FRONT-LEFT
            robot.gyroTurn(MainRobot.TURN_SPEED,0,this); //PARALLEL TO LEFT WALL*/
        }

        //---------------- CASE ONE RING ----------------
        else if(rings == 1){
            robot.encoderDrive(0.3,6,6,6,6,this);

            //POWER-SHOT ONE
            robot.jHopper2.setPower(-0.785);
            robot.gyroTurn(MainRobot.TURN_SPEED,-28.6,this); //TURNS TO PWR SHOT RIGHT
            sleep(1700);

            robot.JHopFlap.setPosition(0.5); //FLAP OPENS
            robot.jHopper1.setPower(-1);
            robot.foamWheel.setPower(1);
            sleep(1000);

            robot.JHopFlap.setPosition(0.1); //FLAP CLOSES
            robot.jHopper1.setPower(0);
            robot.foamWheel.setPower(0);

            //POWER-SHOT TWO
            robot.gyroTurn(MainRobot.TURN_SPEED,-21,this); //TURNS TO PWR SHOT MIDDLE
            robot.JHopFlap.setPosition(0.5); //FLAP OPENS
            robot.jHopper1.setPower(-1);
            robot.foamWheel.setPower(1);
            sleep(1000);

            robot.JHopFlap.setPosition(0.1); //FLAP CLOSES
            robot.jHopper1.setPower(0);
            robot.foamWheel.setPower(0);

            //POWER-SHOT THREE
            robot.gyroTurn(MainRobot.TURN_SPEED,-19.8,this); //TURNS TO PWR SHOT LEFT
            robot.JHopFlap.setPosition(0.5); //FLAP OPENS
            robot.jHopper1.setPower(-1);
            robot.foamWheel.setPower(1);
            sleep(1000);

            robot.JHopFlap.setPosition(0.1); //FLAP CLOSES
            /*robot.jHopper1.setPower(0);
            robot.foamWheel.setPower(0);*/
            robot.jHopper2.setPower(0);

            //WOBBLE & PARKING
            robot.gyroTurn(MainRobot.TURN_SPEED,0,this);
            robot.encoderDrive(0.9,45,45,45,45,this); //STRAFES & COLLECTS RING
            robot.jHopper1.setPower(0);
            robot.foamWheel.setPower(0);
            robot.gyroTurn(MainRobot.TURN_SPEED,0,this);

            robot.wobbleArm.setPower(0.2);
            robot.encoderDrive(0.9,0,8,8,0,this); //DIAGONALLY FRONT-LEFT
            robot.gyroTurn(MainRobot.TURN_SPEED,0,this);
            robot.encoderDrive(MainRobot.DRIVE_SPEED,67,67,67,67,this); //STRAFES FORWARD TO BOX B
            robot.gyroTurn(MainRobot.TURN_SPEED,33,this);
            robot.wobbleArm.setPower(0);

            robot.wobbleClaw.setPower(1);
            sleep(600);
            robot.gyroTurn(MainRobot.TURN_SPEED,0,this);
            robot.wobbleClaw.setPower(0);

            robot.encoderDrive(MainRobot.DRIVE_SPEED,-40,-40,-40,-40,this); //PARKING

            robot.gyroTurn(MainRobot.TURN_SPEED,0,this); //PARALLEL TO LEFT WALL

            //2ND WOBBLE & PARKING (IN PROGRESS)
            /*robot.encoderDrive(MainRobot.DRIVE_SPEED,-135,-135,-135,-135,this);
            robot.encoderDrive(MainRobot.DRIVE_SPEED,30,-30,-30,30,this); //STRAFES RIGHT
            robot.encoderDrive(MainRobot.DRIVE_SPEED,30,30,30,30,this); //STRAFES FORWARD
            robot.encoderDrive(MainRobot.DRIVE_SPEED,0,120,120,0,this); //DIAGONALLY FRONT-LEFT
            robot.gyroTurn(MainRobot.TURN_SPEED,0,this);
            robot.encoderDrive(MainRobot.DRIVE_SPEED,-20,-20,-20,-20,this); //PARKING
            robot.gyroTurn(MainRobot.TURN_SPEED,0,this); //PARALLEL TO LEFT WALL*/
        }

        //---------------- CASE FOUR RINGS ----------------
        else if(rings == 4) {
            robot.encoderDrive(0.3,6,6,6,6,this);

            //POWER-SHOT ONE
            robot.jHopper2.setPower(-0.782);
            robot.gyroTurn(MainRobot.TURN_SPEED,-28.3,this); //TURNS TO PWR SHOT RIGHT
            sleep(1700);

            robot.JHopFlap.setPosition(0.5); //FLAP OPENS
            robot.jHopper1.setPower(-1);
            robot.foamWheel.setPower(1);
            sleep(1000);

            robot.JHopFlap.setPosition(0.1); //FLAP CLOSES
            robot.jHopper1.setPower(0);
            robot.foamWheel.setPower(0);

            //POWER-SHOT TWO
            robot.gyroTurn(MainRobot.TURN_SPEED,-21,this); //TURNS TO PWR SHOT MIDDLE
            robot.JHopFlap.setPosition(0.5); //FLAP OPENS
            robot.jHopper1.setPower(-1);
            robot.foamWheel.setPower(1);
            sleep(1000);

            robot.JHopFlap.setPosition(0.1); //FLAP CLOSES
            robot.jHopper1.setPower(0);
            robot.foamWheel.setPower(0);

            //POWER-SHOT THREE
            robot.gyroTurn(MainRobot.TURN_SPEED,-19.9,this); //TURNS TO PWR SHOT LEFT
            robot.JHopFlap.setPosition(0.5); //FLAP OPENS
            robot.jHopper1.setPower(-1);
            robot.foamWheel.setPower(1);
            sleep(1000);

            robot.JHopFlap.setPosition(0.1); //FLAP CLOSES
            robot.jHopper1.setPower(0);
            robot.foamWheel.setPower(0);
            robot.jHopper2.setPower(0);

            //WOBBLE & PARKING
            robot.gyroTurn(MainRobot.TURN_SPEED,0,this);
            robot.encoderDrive(0.9,0,36,36,0,this); //DIAGONALLY FRONT-LEFT
            robot.gyroTurn(MainRobot.TURN_SPEED,0,this);

            robot.wobbleArm.setPower(0.2);
            robot.encoderDrive(MainRobot.DRIVE_SPEED,128,128,128,128,this);
            robot.wobbleArm.setPower(0);
            robot.gyroTurn(MainRobot.TURN_SPEED,-140,this);

            robot.wobbleClaw.setPower(1);
            sleep(600);
            robot.wobbleClaw.setPower(0);
            robot.gyroTurn(MainRobot.TURN_SPEED,-180,this);

            robot.encoderDrive(MainRobot.DRIVE_SPEED,55,55,55,55,this); //PARKING

            robot.gyroTurn(MainRobot.TURN_SPEED,6.3,this); //PARALLEL TO LEFT WALL


            //2ND WOBBLE & PARKING (IN PROGRESS)
            /*robot.encoderDrive(MainRobot.DRIVE_SPEED,-160,-160,-160,-160,this);
            robot.encoderDrive(MainRobot.DRIVE_SPEED,40,-40,-40,40,this); //STRAFES RIGHT
            robot.encoderDrive(MainRobot.DRIVE_SPEED,35,35,35,35,this); //STRAFES FORWARD
            robot.encoderDrive(MainRobot.DRIVE_SPEED,0,130,130,0,this); //DIAGONALLY FRONT-LEFT
            robot.gyroTurn(MainRobot.TURN_SPEED,0,this);
            robot.encoderDrive(MainRobot.DRIVE_SPEED,-55,-55,-55,-55,this); //PARKING
            robot.gyroTurn(MainRobot.TURN_SPEED,0,this); //PARALLEL TO LEFT WALL*/
        }
    }
}
