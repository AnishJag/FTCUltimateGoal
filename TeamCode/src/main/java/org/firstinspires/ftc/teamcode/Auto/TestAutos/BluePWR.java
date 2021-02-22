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

            robot.gyroDrive(MainRobot.DRIVE_SPEED,-5,-5,-5,-5,0,0,0,0,0,this);

            //POWER-SHOT ONE
            robot.jHopper2.setPower(-1);
            robot.gyroTurn(robot.TURN_SPEED,160,this); //TURNS TO PWR SHOT RIGHT
            sleep(800);

            robot.JHopFlap.setPosition(0.5); //FLAP OPENS
            robot.jHopper1.setPower(-1);
            robot.foamWheel.setPower(1);
            sleep(400);

            robot.JHopFlap.setPosition(0.1); //FLAP CLOSES
            robot.jHopper1.setPower(0);
            robot.foamWheel.setPower(0);

            //POWER-SHOT TWO
            robot.gyroTurn(MainRobot.TURN_SPEED,163,this); //TURNS TO PWR SHOT MIDDLE
            robot.JHopFlap.setPosition(0.5); //FLAP OPENS
            robot.jHopper1.setPower(-1);
            robot.foamWheel.setPower(1);
            sleep(400);

            robot.JHopFlap.setPosition(0.1); //FLAP CLOSES
            robot.jHopper1.setPower(0);
            robot.foamWheel.setPower(0);

            //POWER-SHOT THREE
            robot.gyroTurn(MainRobot.TURN_SPEED,166,this); //TURNS TO PWR SHOT LEFT
            robot.JHopFlap.setPosition(0.5); //FLAP OPENS
            robot.jHopper1.setPower(-1);
            robot.foamWheel.setPower(1);
            sleep(400);

            robot.JHopFlap.setPosition(0.1); //FLAP CLOSES
            robot.jHopper1.setPower(0);
            robot.foamWheel.setPower(0);

            //WOBBLE & PARKING
            robot.gyroTurn(MainRobot.TURN_SPEED,0,this);

            robot.encoderDrive(MainRobot.DRIVE_SPEED,10,-10,-10,10,0,this); //STRAFES RIGHT

            robot.wobbleArm.setPower(0.1);
            robot.gyroDrive(MainRobot.DRIVE_SPEED,-85,-85,-85,-85,0,0,0,0,0,this); //MOVES TO DEPOT/PARKING
            robot.wobbleArm.setPower(0);

            robot.wobbleClaw.setPower(1);
            sleep(600);
            robot.wobbleClaw.setPower(0);

            robot.gyroTurn(MainRobot.TURN_SPEED,180,this); //PERPENDICULAR TO LEFT WALL
        }

        //---------------- CASE ONE RING ----------------
        else if(rings == 1){

            robot.gyroDrive(MainRobot.DRIVE_SPEED,-5,-5,-5,-5,0,0,0,0,0,this);

            //POWER-SHOT ONE
            robot.jHopper2.setPower(-1);
            robot.gyroTurn(robot.TURN_SPEED,160,this); //TURNS TO PWR SHOT RIGHT
            sleep(800);

            robot.JHopFlap.setPosition(0.5); //FLAP OPENS
            robot.jHopper1.setPower(-1);
            robot.foamWheel.setPower(1);
            sleep(400);

            robot.JHopFlap.setPosition(0.1); //FLAP CLOSES
            robot.jHopper1.setPower(0);
            robot.foamWheel.setPower(0);

            //POWER-SHOT TWO
            robot.gyroTurn(MainRobot.TURN_SPEED,163,this); //TURNS TO PWR SHOT MIDDLE
            robot.JHopFlap.setPosition(0.5); //FLAP OPENS
            robot.jHopper1.setPower(-1);
            robot.foamWheel.setPower(1);
            sleep(400);

            robot.JHopFlap.setPosition(0.1); //FLAP CLOSES
            robot.jHopper1.setPower(0);
            robot.foamWheel.setPower(0);

            //POWER-SHOT THREE
            robot.gyroTurn(MainRobot.TURN_SPEED,166,this); //TURNS TO PWR SHOT LEFT
            robot.JHopFlap.setPosition(0.5); //FLAP OPENS
            robot.jHopper1.setPower(-1);
            robot.foamWheel.setPower(1);
            sleep(400);

            robot.JHopFlap.setPosition(0.1); //FLAP CLOSES
            robot.jHopper1.setPower(0);
            robot.foamWheel.setPower(0);

            //WOBBLE & PARKING
            robot.gyroTurn(MainRobot.TURN_SPEED,0,this);

            robot.encoderDrive(MainRobot.DRIVE_SPEED,10,-10,-10,10,0,this); //STRAFES RIGHT

            robot.gyroDrive(MainRobot.DRIVE_SPEED,-50,-50,-50,-50,0,0,0,0,0,this); //MOVES TO DEPOT

            robot.wobbleArm.setPower(0.1);
            robot.encoderDrive(MainRobot.DRIVE_SPEED,-60,0,0,-60,0,this); //STRAFES BACK LEFT
            robot.wobbleArm.setPower(0);

            robot.wobbleClaw.setPower(1);
            sleep(600);
            robot.wobbleClaw.setPower(0);

            robot.encoderDrive(MainRobot.DRIVE_SPEED,25,25,25,25,0,this); //PARKING

            robot.gyroTurn(MainRobot.TURN_SPEED,180,this); //PERPENDICULAR TO LEFT WALL
        }

        //---------------- CASE FOUR RINGS ----------------
        else if(rings == 4) {

            robot.gyroDrive(MainRobot.DRIVE_SPEED,-5,-5,-5,-5,0,0,0,0,0,this);

            //POWER-SHOT ONE
            robot.jHopper2.setPower(-1);
            robot.gyroTurn(robot.TURN_SPEED,160,this); //TURNS TO PWR SHOT RIGHT
            sleep(800);

            robot.JHopFlap.setPosition(0.5); //FLAP OPENS
            robot.jHopper1.setPower(-1);
            robot.foamWheel.setPower(1);
            sleep(400);

            robot.JHopFlap.setPosition(0.1); //FLAP CLOSES
            robot.jHopper1.setPower(0);
            robot.foamWheel.setPower(0);

            //POWER-SHOT TWO
            robot.gyroTurn(MainRobot.TURN_SPEED,163,this); //TURNS TO PWR SHOT MIDDLE
            robot.JHopFlap.setPosition(0.5); //FLAP OPENS
            robot.jHopper1.setPower(-1);
            robot.foamWheel.setPower(1);
            sleep(400);

            robot.JHopFlap.setPosition(0.1); //FLAP CLOSES
            robot.jHopper1.setPower(0);
            robot.foamWheel.setPower(0);

            //POWER-SHOT THREE
            robot.gyroTurn(MainRobot.TURN_SPEED,166,this); //TURNS TO PWR SHOT LEFT
            robot.JHopFlap.setPosition(0.5); //FLAP OPENS
            robot.jHopper1.setPower(-1);
            robot.foamWheel.setPower(1);
            sleep(400);

            robot.JHopFlap.setPosition(0.1); //FLAP CLOSES
            robot.jHopper1.setPower(0);
            robot.foamWheel.setPower(0);

            //WOBBLE & PARKING
            robot.gyroTurn(MainRobot.TURN_SPEED,0,this);

            robot.encoderDrive(MainRobot.DRIVE_SPEED,10,-10,-10,10,0,this); //STRAFES RIGHT

            robot.wobbleArm.setPower(0.1);
            robot.gyroDrive(MainRobot.DRIVE_SPEED,-120,-120,-120,-120,0,0,-0,0,0,this); //MOVES TO DEPOT/PARKING
            robot.wobbleArm.setPower(0);

            robot.wobbleClaw.setPower(1);
            sleep(600);
            robot.wobbleClaw.setPower(0);

            robot.encoderDrive(MainRobot.DRIVE_SPEED,70,70,70,70,0,this); //PARKING

            robot.gyroTurn(MainRobot.TURN_SPEED,180,this); //PERPENDICULAR TO LEFT WALL
        }
    }
}
