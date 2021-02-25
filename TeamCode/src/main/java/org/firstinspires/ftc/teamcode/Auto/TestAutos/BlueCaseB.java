package org.firstinspires.ftc.teamcode.Auto.TestAutos;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.RingDetector;
import org.firstinspires.ftc.teamcode.MainRobot;


@Autonomous(name="BlueCaseB")
@Disabled
public class BlueCaseB extends LinearOpMode {

    MainRobot robot = new MainRobot();

    @Override
    public void runOpMode() throws InterruptedException {

        RingDetector detector = new RingDetector(this, false);

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

        robot.encoderDrive(MainRobot.DRIVE_SPEED,0,-50,-50,0,this);
        robot.gyroDrive(MainRobot.DRIVE_SPEED,-98,-98,-98,-98,0,0,0,0,0,this);

        robot.gyroTurn(robot.TURN_SPEED,-90,this);

        robot.wobbleArm.setPower(0.40);
        sleep(75);

        robot.wobbleClaw.setPower(0.5);
        sleep(1600);

        //SHOOTING & PARK

        robot.encoderDrive(MainRobot.DRIVE_SPEED,-27,27,27,-27,this); //STRAFES LEFT

        robot.wobbleArm.setPower(0);
        robot.wobbleClaw.setPower(0);
        //robot.jhopper1.setPower(0);
        //robot.jhopper2.setPower(0);
        //robot.JHopFlap.setPosition(0.1);
    }
}
