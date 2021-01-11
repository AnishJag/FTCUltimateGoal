package org.firstinspires.ftc.teamcode.Auto.TestAutos;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MainRobot;

// After testing this program, robot kept curving towards the right.

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

        sleep(6000);

        telemetry.addData("Top-Left Power: ", robot.topLeft.getPower());
        telemetry.addData("Top-Right Power: ", robot.topRight.getPower());
        telemetry.addData("Back-Right Power: ", robot.bottomRight.getPower());
        telemetry.addData("Back-Right Power: ", robot.bottomLeft.getPower());
        telemetry.update();

    }
}
