package org.firstinspires.ftc.teamcode.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MainRobot;


@Autonomous(name="Blue")
public class Blue extends LinearOpMode {

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

        int rings = detector.getDecision();

        if (rings == 0){

            robot.gyroDrive(MainRobot.DRIVE_SPEED,3,3,3,3,0,5,5,-1,this);
            //robot.gyroTurn(robot.TURN_SPEED,3,this);
            robot.wobbleArm.setPower(1);
            sleep(1000);
            robot.wobbleArm.setPower(-1);
            sleep(1000);
            robot.wobbleArm.setPower(0);

        }
        else if(rings == 1){

            robot.gyroDrive(MainRobot.DRIVE_SPEED,3,3,3,3,0,5,5,-1,this);
        }
        else if(rings == 4) {

            robot.gyroDrive(MainRobot.DRIVE_SPEED, 3, 3, 3, 3, 0, 5,5,-1,this);

        }

        robot.gyroTurn(MainRobot.TURN_SPEED,180,this); // Ensures the robot is perpendicular from left wall.
    }

}
