package org.firstinspires.ftc.teamcode.Auto.TestAutos;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.RingDetector;
import org.firstinspires.ftc.teamcode.MainRobot;


@Autonomous(name="BlueCaseB")
public class BlueCaseB extends LinearOpMode {

    MainRobot robot = new MainRobot();

    @Override
    public void runOpMode() throws InterruptedException {

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


        robot.encoderDrive(MainRobot.DRIVE_SPEED, 0, -65, -65, 0,this);
    }
}
