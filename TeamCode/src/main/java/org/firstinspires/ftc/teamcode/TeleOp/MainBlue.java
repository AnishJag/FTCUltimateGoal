package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Auto.RingDetectorJHop;
import org.firstinspires.ftc.teamcode.MainRobot;

import java.lang.reflect.Field;

@TeleOp(name="MainTeleOp BLUE")
public class
MainBlue extends OpMode{

    MainRobot robot = null;

    public static final double BACK_ALIGNMENT = 51;
    public static final double LEFT_ALIGNMENT = 39;
    public static final double MARGIN_OF_ERROR = 1;
    public boolean                 GP1_RB_Held = false;
    public boolean                 SlowMode    = false;
    public boolean               FieldRelative = true;
    public boolean                 GP1_LB_Held = false;
    public boolean                GP2_DPL_Held = false;

    //RingDetectorJHop detectorJHop = new RingDetectorJHop(this);

    @Override
    public void init() {
        robot = new MainRobot();
        robot.init(hardwareMap);
        telemetry.addData("Initialization Complete!","");
        telemetry.update();
    }

    @Override
    public void loop() {

        //--------------------DRIVE-TRAIN CONTROLS--------------------\\
        double forward = -gamepad1.left_stick_y;
        double right   =  gamepad1.left_stick_x;

        if(FieldRelative){
            double angle = robot.gyro.getHeading() + 180;
            angle = Math.toRadians(angle);

            double relativeForward = (Math.cos(angle)*right) + (Math.sin(angle)*forward);
            double relativeRight = (Math.sin(angle)*right) - (Math.cos(angle)*forward);

            forward = relativeForward;
            right   = relativeRight;
        }

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

        //--------------------FIELD-RELATIVE DRIVE--------------------\\
        if(gamepad1.left_bumper && !GP1_LB_Held){
            FieldRelative = !FieldRelative;
            GP1_LB_Held = true;
        }
        if(!gamepad1.left_bumper){
            GP1_LB_Held = false;
        }


        //--------------------SLOW-MODE--------------------\\
        if(gamepad1.right_bumper && !GP1_RB_Held){
            GP1_RB_Held = true;
            SlowMode    =  !SlowMode;
        }
        if(!gamepad1.right_bumper){
            GP1_RB_Held = false;
        }

        if(SlowMode) {
            robot.topLeft.setPower(0.3 * leftFrontPower);
            robot.bottomLeft.setPower(0.3 * leftBackPower);
            robot.topRight.setPower(0.3 * rightFrontPower);
            robot.bottomRight.setPower(0.3 * rightBackPower);
        }
        else{
            robot.topLeft.setPower(leftFrontPower);
            robot.bottomLeft.setPower(leftBackPower);
            robot.topRight.setPower(rightFrontPower);
            robot.bottomRight.setPower(rightBackPower);
        }


        //--------------------AUTOMATION CONTROLS--------------------\\
        boolean jHopperWheels = false;

        //---------------TOP-GOAL & SHOOT---------------\\
        if(gamepad1.a){
            double frontRange = robot.frontRange.getDistance(DistanceUnit.INCH);
            double leftRange   = robot.leftRange.getDistance(DistanceUnit.INCH);
            telemetry.addData("Front Range: ", frontRange);
            telemetry.addData("Left Range: ", leftRange);

            double forwardPower = 0;
            double rightPower    = 0;
            if (frontRange < BACK_ALIGNMENT - MARGIN_OF_ERROR){
                forwardPower = -1;
            }
            else if(frontRange > BACK_ALIGNMENT + MARGIN_OF_ERROR){
                forwardPower = 1;
            }
            if(leftRange < LEFT_ALIGNMENT - MARGIN_OF_ERROR){
                rightPower = 1;
            }
            else if(leftRange > LEFT_ALIGNMENT + MARGIN_OF_ERROR){
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

        //---------------POWER-SHOT & SHOOT---------------\\
        if(gamepad1.b){
            double frontRange = robot.frontRange.getDistance(DistanceUnit.INCH);
            double leftRange   = robot.leftRange.getDistance(DistanceUnit.INCH);
            telemetry.addData("Front Range: ", frontRange);
            telemetry.addData("Left Range: ", leftRange);

            double forwardPower = 0;
            double rightPower    = 0;
            if (frontRange < BACK_ALIGNMENT - MARGIN_OF_ERROR){
                forwardPower = -1;
            }
            else if(frontRange > BACK_ALIGNMENT + MARGIN_OF_ERROR){
                forwardPower = 1;
            }
            if(leftRange < LEFT_ALIGNMENT - MARGIN_OF_ERROR){
                rightPower = 1;
            }
            else if(leftRange > LEFT_ALIGNMENT + MARGIN_OF_ERROR){
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

        //---------------TOP-GOAL NO SHOOT---------------\\
        if(gamepad1.x){
            double frontRange = robot.frontRange.getDistance(DistanceUnit.INCH);
            double leftRange   = robot.leftRange.getDistance(DistanceUnit.INCH);
            telemetry.addData("Front Range: ", frontRange);
            telemetry.addData("Left Range: ", leftRange);

            double forwardPower = 0;
            double rightPower    = 0;
            if (frontRange < BACK_ALIGNMENT - MARGIN_OF_ERROR){
                forwardPower = -1;
            }
            else if(frontRange > BACK_ALIGNMENT + MARGIN_OF_ERROR){
                forwardPower = 1;
            }
            if(leftRange < LEFT_ALIGNMENT - MARGIN_OF_ERROR){
                rightPower = 1;
            }
            else if(leftRange > LEFT_ALIGNMENT + MARGIN_OF_ERROR){
                rightPower = -1;
            }
            robot.topLeft.setPower(forwardPower + rightPower);
            robot.topRight.setPower(forwardPower - rightPower);
            robot.bottomLeft.setPower(forwardPower - rightPower);
            robot.bottomRight.setPower(forwardPower + rightPower);
        }

        //---------------POWER-SHOT NO SHOOT---------------\\
        if(gamepad1.y){
            double frontRange = robot.frontRange.getDistance(DistanceUnit.INCH);
            double leftRange   = robot.leftRange.getDistance(DistanceUnit.INCH);
            telemetry.addData("Front Range: ", frontRange);
            telemetry.addData("Left Range: ", leftRange);

            double forwardPower = 0;
            double rightPower    = 0;
            if (frontRange < BACK_ALIGNMENT - MARGIN_OF_ERROR){
                forwardPower = -1;
            }
            else if(frontRange > BACK_ALIGNMENT + MARGIN_OF_ERROR){
                forwardPower = 1;
            }
            if(leftRange < LEFT_ALIGNMENT - MARGIN_OF_ERROR){
                rightPower = 1;
            }
            else if(leftRange > LEFT_ALIGNMENT + MARGIN_OF_ERROR){
                rightPower = -1;
            }
            robot.topLeft.setPower(forwardPower + rightPower);
            robot.topRight.setPower(forwardPower - rightPower);
            robot.bottomLeft.setPower(forwardPower - rightPower);
            robot.bottomRight.setPower(forwardPower + rightPower);
            //Need to add angle for each turn for power-shots.
        }


        //--------------------ROBOT CONTROLS--------------------\\

        //---------------FOAM WHEEL---------------\\
        if (gamepad2.a || gamepad2.left_bumper || gamepad2.right_bumper){
            robot.foamWheel.setPower(1);
        }
        else if (gamepad2.dpad_down){
            robot.foamWheel.setPower(-1);
        }
        else{
            robot.foamWheel.setPower(0);
        }

        //---------------J-HOPPER 1---------------\\
        if (gamepad2.b || gamepad2.left_bumper || gamepad2.right_bumper || jHopperWheels){
            robot.jHopper1.setPower(-1);
        }
        else if (gamepad2.dpad_right){
            robot.jHopper1.setPower(1);
        }
        else{
            robot.jHopper1.setPower(0);
        }

        //---------------J-HOPPER 2---------------\\
        if (gamepad2.y || gamepad2.left_bumper || gamepad2.right_bumper || jHopperWheels){
            robot.jHopper2.setPower(-0.95);
        }
        else if (gamepad2.dpad_up){
            robot.jHopper2.setPower(0.3);
        }
        else{
            robot.jHopper2.setPower(0);
        }
        telemetry.addData("J-Hopper 2 Power: ", robot.jHopper2.getPower());
        telemetry.update();

        //---------------WOBBLE---------------\\
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

        //---------------J-HOPPER FLAP---------------\\
        if(gamepad2.dpad_left && !GP2_DPL_Held) {
            GP2_DPL_Held = true;
            if(robot.JHopFlap.getPosition() == 0){
                robot.JHopFlap.setPosition(0.5);
            }
            else{
                robot.JHopFlap.setPosition(0.5);
            }
        }
        if(!gamepad2.dpad_left) {
            GP2_DPL_Held = false; //Back Open
        }
        telemetry.update();
    }
}
