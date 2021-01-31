package org.firstinspires.ftc.teamcode.TeleOp.TestTeleOps;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.Auto.RingDetectorJHop;
import org.firstinspires.ftc.teamcode.MainRobot;

import java.lang.reflect.Field;

@TeleOp(name="Encoder Test")

public class EncoderTest extends OpMode {

    MainRobot robot = null;


    @Override
    public void init() {
        robot = new MainRobot();
        robot.init(hardwareMap);

        telemetry.addData("Initialization Complete!", "");
        telemetry.update();
    }

    @Override
    public void loop() {

        telemetry.addData("FR Encoder: ", robot.topRight.getCurrentPosition());
        telemetry.addData("FL Encoder: ", robot.topLeft.getCurrentPosition());
        telemetry.addData("BR Encoder: ", robot.bottomRight.getCurrentPosition());
        telemetry.addData("BL Encoder: ", robot.bottomLeft.getCurrentPosition());
        telemetry.update();
    }
}