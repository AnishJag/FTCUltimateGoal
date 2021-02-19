package org.firstinspires.ftc.teamcode.Auto.TestAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MainRobot;


@Autonomous(name="testJhopper")
@Disabled
public class JHopperTest extends LinearOpMode {

    MainRobot robot = new MainRobot();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        waitForStart();

        robot.jHopper1.setPower(-1);
        sleep(8000);

        robot.jHopper2.setPower(-0.8);
        sleep(8000);

        telemetry.addData("J-Hopper1 Power: ", robot.jHopper1.getPower());
        telemetry.addData("J-Hopper2 Power: ", robot.jHopper2.getPower());
        telemetry.update();
    }
}