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

    }
}