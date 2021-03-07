package org.firstinspires.ftc.teamcode.Auto.TestAutos.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MainRobot;


@Autonomous(name="testJhopper")
@Disabled
public class JHopperTest extends LinearOpMode {

    MainRobot robot = new MainRobot();

    public boolean                   firstTime = true;
    public double                            initTime;
    public int                                initPos;


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        robot.jHopper2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        robot.jHopper2.setPower(1);
        while(opModeIsActive()) {
            if (firstTime || robot.getCurrentRPM(initTime, time, initPos, robot.jHopper2.getCurrentPosition(), this)) {
                initTime = this.time;
                initPos = robot.jHopper2.getCurrentPosition();
                firstTime = false;
            }
        }
    }
}