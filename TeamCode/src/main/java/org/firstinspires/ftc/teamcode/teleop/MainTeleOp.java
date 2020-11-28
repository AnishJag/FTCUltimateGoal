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

    }
}
