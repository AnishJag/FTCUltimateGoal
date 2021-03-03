package org.firstinspires.ftc.teamcode.Auto.TestAutos.Tests;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.RingDetector;
import org.firstinspires.ftc.teamcode.MainRobot;


@Autonomous(name="Functions Test")
public class FunctionsTest extends LinearOpMode {

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

        //Gyro & Range Test 1
        /*robot.gyroDrive(MainRobot.DRIVE_SPEED,50,50,50,50,0,0,0,0,0,this);
        //robot.rangeDrive(0.3,-1,-1,-1,this);

        //Gyro & Range Test 2
        robot.gyroDrive(MainRobot.DRIVE_SPEED,50,0,0,50,0,0,0,0,0,this);
        robot.rangeDrive(0.3,-1,-1,-1,this);

        //Gyro & Range Test 3
        robot.gyroDrive(MainRobot.DRIVE_SPEED,0,50,50,0,0,0,0,0,0,this);
        robot.rangeDrive(0.3,-1,-1,-1,this);

        //Encoder & Range Test
        robot.encoderDrive(MainRobot.DRIVE_SPEED,60,60,60,60,this);
        robot.rangeDrive(0.3,-1,-1,-1,this);*/

    }
}
