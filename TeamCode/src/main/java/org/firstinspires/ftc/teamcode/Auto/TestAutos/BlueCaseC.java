package org.firstinspires.ftc.teamcode.Auto.TestAutos;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.RingDetector;
import org.firstinspires.ftc.teamcode.MainRobot;


@Autonomous(name="BlueCaseC")
@Disabled
public class BlueCaseC extends LinearOpMode {

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

        robot.gyro.resetZAxisIntegrator();


            robot.encoderDrive(MainRobot.DRIVE_SPEED, 0, -50, -50, 0, this);
            robot.gyroDrive(MainRobot.DRIVE_SPEED,-112,-112,-112,-112,0,0,0,0,0,this);
            robot.gyroTurn(MainRobot.TURN_SPEED,-4,this);

            robot.wobbleArm.setPower(0.5);
            robot.sleepV2(0.045,this);

            robot.wobbleClaw.setPower(0.8);
            robot.sleepV2(1.2,this);

            robot.gyroDrive(MainRobot.DRIVE_SPEED,25,25,25,25,0,0,0,0,0,this); //PARKING

            telemetry.addLine("I'M STUCK");
            telemetry.update();

            robot.wobbleArm.setPower(0);
            robot.wobbleClaw.setPower(0);

            //SHOOTING AND PARKING
            /*telemetry.addLine("I SHOULD START STRAFING!");
            telemetry.update();
            robot.encoderDrive(MainRobot.DRIVE_SPEED, 0, 150, 150, 0, this); //STRAFES DIAGONALLY LEFT
            telemetry.addLine("I SHOULD TURN");
            telemetry.update();
            robot.gyroTurn(robot.TURN_SPEED, 178, this);
            robot.gyroDrive(0.6, 1, 1, 1, 1, 0, 67, 42, -1,0,0,0,0,this);

            robot.jHopper2.setPower(-0.95);
            sleep(2500);
            /*while (detectorJHop.getDecision() > 0){
                robot.JHopFlap.setPosition(0.5);
                robot.jHopper1.setPower(-1);
                robot.foamWheel.setPower(1);
            }*/
            /*robot.JHopFlap.setPosition(0.5);
            robot.jHopper1.setPower(-1);
            robot.foamWheel.setPower(1);
            robot.sleepV2(3.4, this);

            robot.jHopper1.setPower(0);
            robot.jHopper2.setPower(0);
            robot.foamWheel.setPower(0);
            robot.JHopFlap.setPosition(0.1);*/

            robot.gyroDrive(MainRobot.DRIVE_SPEED,25,25,25,25,0,0,0,0,0,this); //PARKING
    }
}
