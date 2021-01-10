package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MainRobot;

@TeleOp(name="MainTeleOp")

public class MainBlue extends OpMode{

    MainRobot robot = null;

    public static final double FRONT_ALIGNMENT = 68;
    public static final double RIGHT_ALIGNMENT = 36;
    public static final double MARGIN_OF_ERROR = 1;

    @Override
    public void init() {
        robot = new MainRobot();
        robot.init(hardwareMap);
        telemetry.addData("Initialization Complete!","");
        telemetry.update();
    }

    @Override
    public void loop() {

        //--------------------DRIVETRAIN CONTROLS--------------------\\
        double forward = -gamepad1.left_stick_y;
        double right   = gamepad1.left_stick_x;
        double turn    = gamepad1.right_stick_x;

        double leftFrontPower = forward + right + turn;
        double leftBackPower = forward - right + turn;
        double rightFrontPower = forward - right - turn;
        double rightBackPower = forward + right - turn;
        double[] powers = {leftFrontPower, leftBackPower, rightFrontPower, rightBackPower};

        boolean needToScale = false;
        for (double power : powers){
            if(Math.abs(power) > 1){
                needToScale = true;
                break;
            }
        }
        if (needToScale){
            double greatest = 0;
            for (double power : powers){
                if (Math.abs(power) > greatest){
                    greatest = Math.abs(power);
                }
            }
            leftFrontPower /= greatest;
            leftBackPower /= greatest;
            rightFrontPower /= greatest;
            rightBackPower /= greatest;
        }
        robot.topLeft.setPower(leftFrontPower);
        robot.bottomLeft.setPower(leftBackPower);
        robot.topRight.setPower(rightFrontPower);
        robot.bottomRight.setPower(rightBackPower);

        //--------------------AUTOMATION CONTROLS--------------------\\
        boolean jHopperWheels = false;

        if(gamepad1.a){
            double frontRange = robot.frontRange.getDistance(DistanceUnit.INCH);
            double leftRange   = robot.leftRange.getDistance(DistanceUnit.INCH);
            telemetry.addData("Front Range: ", frontRange);
            telemetry.addData("Left Range: ", leftRange);

            double forwardPower = 0;
            double rightPower    = 0;
            if (frontRange < FRONT_ALIGNMENT - MARGIN_OF_ERROR){
                forwardPower = -1;
            }
            else if(frontRange > FRONT_ALIGNMENT + MARGIN_OF_ERROR){
                forwardPower = 1;
            }
            if(leftRange < RIGHT_ALIGNMENT - MARGIN_OF_ERROR){
                rightPower = 1;
            }
            else if(leftRange > RIGHT_ALIGNMENT + MARGIN_OF_ERROR){
                rightPower = -1;
            }
            robot.topLeft.setPower(forwardPower + rightPower);
            robot.topRight.setPower(forwardPower - rightPower);
            robot.bottomLeft.setPower(forwardPower - rightPower);
            robot.bottomRight.setPower(forwardPower + rightPower);

            jHopperWheels = true;
        }
        else{
            jHopperWheels = false;
        }

        if(gamepad1.b){
            double frontRange = robot.frontRange.getDistance(DistanceUnit.INCH);
            double leftRange   = robot.leftRange.getDistance(DistanceUnit.INCH);
            telemetry.addData("Front Range: ", frontRange);
            telemetry.addData("Left Range: ", leftRange);

            double forwardPower = 0;
            double rightPower    = 0;
            if (frontRange < FRONT_ALIGNMENT - MARGIN_OF_ERROR){
                forwardPower = -1;
            }
            else if(frontRange > FRONT_ALIGNMENT + MARGIN_OF_ERROR){
                forwardPower = 1;
            }
            if(leftRange < RIGHT_ALIGNMENT - MARGIN_OF_ERROR){
                rightPower = 1;
            }
            else if(leftRange > RIGHT_ALIGNMENT + MARGIN_OF_ERROR){
                rightPower = -1;
            }
            robot.topLeft.setPower(forwardPower + rightPower);
            robot.topRight.setPower(forwardPower - rightPower);
            robot.bottomLeft.setPower(forwardPower - rightPower);
            robot.bottomRight.setPower(forwardPower + rightPower);

            jHopperWheels = true;
            //Need to add angle for each turn for power-shots.
        }
        else{
            jHopperWheels = false;
        }

        if(gamepad1.x){
            double frontRange = robot.frontRange.getDistance(DistanceUnit.INCH);
            double leftRange   = robot.leftRange.getDistance(DistanceUnit.INCH);
            telemetry.addData("Front Range: ", frontRange);
            telemetry.addData("Left Range: ", leftRange);

            double forwardPower = 0;
            double rightPower    = 0;
            if (frontRange < FRONT_ALIGNMENT - MARGIN_OF_ERROR){
                forwardPower = -1;
            }
            else if(frontRange > FRONT_ALIGNMENT + MARGIN_OF_ERROR){
                forwardPower = 1;
            }
            if(leftRange < RIGHT_ALIGNMENT - MARGIN_OF_ERROR){
                rightPower = 1;
            }
            else if(leftRange > RIGHT_ALIGNMENT + MARGIN_OF_ERROR){
                rightPower = -1;
            }
            robot.topLeft.setPower(forwardPower + rightPower);
            robot.topRight.setPower(forwardPower - rightPower);
            robot.bottomLeft.setPower(forwardPower - rightPower);
            robot.bottomRight.setPower(forwardPower + rightPower);
        }

        if(gamepad1.y){
            double frontRange = robot.frontRange.getDistance(DistanceUnit.INCH);
            double leftRange   = robot.leftRange.getDistance(DistanceUnit.INCH);
            telemetry.addData("Front Range: ", frontRange);
            telemetry.addData("Left Range: ", leftRange);

            double forwardPower = 0;
            double rightPower    = 0;
            if (frontRange < FRONT_ALIGNMENT - MARGIN_OF_ERROR){
                forwardPower = -1;
            }
            else if(frontRange > FRONT_ALIGNMENT + MARGIN_OF_ERROR){
                forwardPower = 1;
            }
            if(leftRange < RIGHT_ALIGNMENT - MARGIN_OF_ERROR){
                rightPower = 1;
            }
            else if(leftRange > RIGHT_ALIGNMENT + MARGIN_OF_ERROR){
                rightPower = -1;
            }
            robot.topLeft.setPower(forwardPower + rightPower);
            robot.topRight.setPower(forwardPower - rightPower);
            robot.bottomLeft.setPower(forwardPower - rightPower);
            robot.bottomRight.setPower(forwardPower + rightPower);
            //Need to add angle for each turn for power-shots.
        }


        //--------------------NON-AUTOMATION CONTROLS--------------------\\
        if (gamepad2.a || gamepad2.left_bumper || gamepad2.right_bumper){
            robot.foamWheel.setPower(1);
        }
        else if (gamepad2.dpad_down){
            robot.foamWheel.setPower(-1);
        }
        else{
            robot.foamWheel.setPower(0);
        }

        if (gamepad2.b || gamepad2.left_bumper || gamepad2.right_bumper || jHopperWheels){
            robot.jHopper1.setPower(-1);
        }
        else if (gamepad2.dpad_down){
            robot.jHopper1.setPower(1);
        }
        else{
            robot.jHopper1.setPower(0);
        }

        if (gamepad2.y || gamepad2.left_bumper || gamepad2.right_bumper || jHopperWheels){
            robot.jHopper2.setPower(-0.8);
        }
        else if (gamepad2.dpad_up){
            robot.jHopper2.setPower(1);
        }
        else{
            robot.jHopper2.setPower(0);
        }

        telemetry.addData("J-Hopper1 Power: ", robot.jHopper1.getPower());

        double wobbleArm = gamepad2.right_stick_y;
        if(Math.abs(wobbleArm) < 0.1){
            robot.wobbleArm.setPower(0);
        }
        else{
            robot.wobbleArm.setPower(wobbleArm);
        }
        if(gamepad2.x){
            robot.wobbleClaw.setPower(1);
        }
        else{
            robot.wobbleClaw.setPower(0);
        }

        telemetry.update();
    }
}
