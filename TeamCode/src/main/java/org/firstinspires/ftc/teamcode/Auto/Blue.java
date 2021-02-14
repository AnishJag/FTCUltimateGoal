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

            robot.gyroDrive(MainRobot.DRIVE_SPEED,-99,-99,-99,-99,0,-1,-1,-1,this);

            robot.gyroTurn(robot.TURN_SPEED,83,this);

            robot.wobbleArm.setPower(0.40);
            sleep(75);

            robot.wobbleClaw.setPower(0.5);
            sleep(1600);

            //SHOOTING & PARK
            robot.encoderDrive(MainRobot.DRIVE_SPEED,15,0,0,15,this); //STRAFES DIAGONALLY RIGHT
            robot.gyroDrive(MainRobot.DRIVE_SPEED,3,3,3,3,0,-1,-1,-1,this); //DRIVES TO SHOOTING POSITION
            robot.gyroTurn(robot.TURN_SPEED,0,this); //TURNS TO SHOOTING ANGLE
            robot.gyroDrive(0.5,0,0,0,0,0,28.0,42.0,-1,this);

            robot.jHopper2.setPower(-0.95);
            sleep(4000);
            robot.JHopFlap.setPosition(0.5);
            robot.jHopper1.setPower(1);
            robot.foamWheel.setPower(1);
            sleep(8000);

            robot.jHopper1.setPower(0);
            robot.jHopper2.setPower(0);
            robot.JHopFlap.setPosition(0.1);

            //REVERT POWERS BACK TO ZERO
            robot.wobbleArm.setPower(0);
            robot.wobbleClaw.setPower(0);

            robot.gyroDrive(MainRobot.DRIVE_SPEED,6,6,6,6,0,-1,-1,-1,this); //PARKING
        }
        else if(rings == 1){

            robot.encoderDrive(MainRobot.DRIVE_SPEED,0,-50,-50,0,this);
            robot.gyroDrive(MainRobot.DRIVE_SPEED,-101,-101,-101,-101,0,-1,-1,-1,this);

            robot.gyroTurn(robot.TURN_SPEED,-82,this);

            robot.wobbleArm.setPower(0.40);
            sleep(75);

            robot.wobbleClaw.setPower(0.5);
            sleep(1600);

            //SHOOTING & PARK

            robot.encoderDrive(MainRobot.DRIVE_SPEED,-30,30,30,-30,this); //STRAFES LEFT

            robot.wobbleArm.setPower(0);
            robot.wobbleClaw.setPower(0);
            //robot.jhopper1.setPower(0);
            //robot.jhopper2.setPower(0);
            //robot.JHopFlap.setPosition(0.1);
        }
        else if(rings == 4) {

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
