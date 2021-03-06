package org.firstinspires.ftc.teamcode.TeleOp.TestTeleOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MainRobot;

import java.lang.reflect.Field;

@TeleOp(name="RangeSensor Test")
public class RangeSensorTest extends OpMode{

    MainRobot robot = null;

    @Override
    public void init() {
        robot = new MainRobot();
        robot.init(hardwareMap);
        telemetry.addData("Initialization Complete!","");
        telemetry.update();
    }

    @Override
    public void loop() {
        double frontDistance = robot.frontRange.getDistance(DistanceUnit.INCH);
        double leftDistance  = robot.leftRange.getDistance(DistanceUnit.INCH);
        double rightDistance = robot.rightRange.getDistance(DistanceUnit.INCH);
        double gyroAngle     = robot.gyro.getIntegratedZValue();
        double castedFront   = Math.round(frontDistance * 1000) / 1000.0;
        double castedLeft    = Math.round(leftDistance * 1000) / 1000.0;
        double castedRight   = Math.round(rightDistance * 1000) / 1000.0;


        telemetry.addData("Front Distance: ", castedFront);
        telemetry.addData("Left Distance: ", castedLeft);
        telemetry.addData("Right Distance: ", castedRight);
        telemetry.addData("Gyro Angle: ", gyroAngle);
        telemetry.update();
    }
}

