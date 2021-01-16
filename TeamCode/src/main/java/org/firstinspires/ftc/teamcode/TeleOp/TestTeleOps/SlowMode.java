package org.firstinspires.ftc.teamcode.TeleOp.TestTeleOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MainRobot;

@TeleOp(name="SlowMode Test")
@Disabled
public class SlowMode extends OpMode {

    MainRobot robot = null;

    @Override
    public void init() {
        robot = new MainRobot();
        robot.init(hardwareMap);

    }

    @Override
    public void loop() {

        //--------------------DRIVE-TRAIN CONTROLS--------------------\\
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