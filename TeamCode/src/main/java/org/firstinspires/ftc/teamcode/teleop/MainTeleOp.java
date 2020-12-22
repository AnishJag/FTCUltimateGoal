package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MainRobot;

@TeleOp(name="MainTeleOp")

public class MainTeleOp extends OpMode{

    MainRobot robot = null;

    @Override
    public void init() {
        robot = new MainRobot();
        robot.init(hardwareMap);
        telemetry.addData("Initialization Complete!","");
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


        //--------------------BUMPER CONTROLS--------------------\\
        if(gamepad2.left_bumper){
            robot.foamWheel.setPower(0.6);
            robot.jHopper1.setPower(1);
            robot.jHopper2.setPower(1);
        }
        else{
            robot.foamWheel.setPower(0);
            robot.jHopper1.setPower(0);
            robot.jHopper2.setPower(0);
        }
        if(gamepad2.right_bumper){
            robot.jHopper1.setPower(1);
            robot.jHopper2.setPower(1);
        }
        else{
            robot.jHopper1.setPower(0);
            robot.jHopper2.setPower(0);
        }


        //--------------------BUTTON CONTROLS--------------------\\
        if(gamepad2.a){
            robot.foamWheel.setPower(0.6);
        }
        else{
            robot.foamWheel.setPower(0);
        }
        if(gamepad2.b){
            robot.jHopper1.setPower(1);
        }
        else{
            robot.jHopper1.setPower(0);
        }
        if(gamepad2.y){
            robot.jHopper2.setPower(1);
        }
        else{
            robot.jHopper2.setPower(0);
        }
        double wobbleArm = gamepad2.right_stick_y;
        if(Math.abs(wobbleArm) < 0.1){
            robot.wobbleArm.setPower(0);
        }
        else{
            robot.wobbleArm.setPower(wobbleArm);
        }


        //--------------------REVERSE CONTROLS--------------------\\
        if(gamepad2.dpad_down){
            robot.foamWheel.setPower(-0.4);
        }
        else{
            robot.foamWheel.setPower(0);
        }
        if(gamepad2.dpad_right){
            robot.jHopper1.setPower(-1);
        }
        else{
            robot.jHopper1.setPower(0);
        }
        if(gamepad2.dpad_up){
            robot.jHopper2.setPower(-1);
        }
        else{
            robot.jHopper2.setPower(0);
        }


    }
}
