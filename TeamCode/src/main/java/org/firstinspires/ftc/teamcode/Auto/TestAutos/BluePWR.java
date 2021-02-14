package org.firstinspires.ftc.teamcode.Auto.TestAutos;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.RingDetector;
import org.firstinspires.ftc.teamcode.MainRobot;


@Autonomous(name="BluePWR")
@Disabled
public class BluePWR extends LinearOpMode {

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

            //NOT FINISHED
            robot.gyroDrive(MainRobot.DRIVE_SPEED,-99,-99,-99,-99,0,-1,-1,-1,this);

            robot.gyroTurn(robot.TURN_SPEED,87,this);

            robot.wobbleArm.setPower(0.40);
            sleep(75);

            robot.wobbleClaw.setPower(0.5);
            sleep(1600);

            //SHOOTING & PARK


            robot.gyroDrive(MainRobot.DRIVE_SPEED,6,6,6,6,0,-1,-1,-1,this); //PARKING

            robot.wobbleArm.setPower(0);
            robot.wobbleClaw.setPower(0);
            //robot.jhopper1.setPower(0);
            //robot.jhopper2.setPower(0);
            //robot.JHopFlap.setPosition(0.1);
        }
        else if(rings == 1){

            //NOT FINISHED
            robot.encoderDrive(MainRobot.DRIVE_SPEED,0,-60,-60,0,this);
            robot.gyroDrive(MainRobot.DRIVE_SPEED,-94,-94,-94,-94,0,-1,-1,-1,this);

            robot.gyroTurn(robot.TURN_SPEED,-87,this);

            robot.wobbleArm.setPower(0.40);
            sleep(75);

            robot.wobbleClaw.setPower(0.5);
            sleep(1600);

            //SHOOTING & PARK

            robot.encoderDrive(MainRobot.DRIVE_SPEED,-23,23,23,-23,this);

            robot.wobbleArm.setPower(0);
            robot.wobbleClaw.setPower(0);
            //robot.jhopper1.setPower(0);
            //robot.jhopper2.setPower(0);
            //robot.JHopFlap.setPosition(0.1);
        }
        else if(rings == 4) {

            //NOT FINISHED
            robot.encoderDrive(MainRobot.DRIVE_SPEED,0,-60,-60,0,this);
            robot.gyroDrive(MainRobot.DRIVE_SPEED,-115,-115,-115,-115,0,-1,-1,-1,this);


            robot.wobbleArm.setPower(0.40);
            sleep(90);

            robot.wobbleClaw.setPower(0.8);
            sleep(1200);

            //SHOOTING AND PARK

            robot.gyroDrive(MainRobot.DRIVE_SPEED,29,29,29,29,0,-1,-1,-1,this);

            robot.wobbleArm.setPower(0);
            robot.wobbleClaw.setPower(0);
            //robot.jhopper1.setPower(0);
            //robot.jhopper2.setPower(0);
            //robot.JHopFlap.setPosition(0.1);
        }
    }

}
