package org.firstinspires.ftc.teamcode.auto.testautos;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MainRobot;

@Autonomous(name="DriveTrainTest")
public class DriveTrainTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MainRobot robot = new MainRobot();
        robot.init(hardwareMap);

        waitForStart();

        robot.topLeft.setPower(0.2);
        robot.topRight.setPower(0.2);
        robot.bottomLeft.setPower(0.2);
        robot.bottomRight.setPower(0.2);

        sleep(10000);
    }
}
