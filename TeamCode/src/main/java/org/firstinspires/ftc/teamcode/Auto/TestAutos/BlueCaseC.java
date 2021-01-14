package org.firstinspires.ftc.teamcode.Auto.TestAutos;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Auto.RingDetector;

@Autonomous(name="BlueCaseC")
@Disabled
public class BlueCaseC extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RingDetector detector = new RingDetector(this, false);

        waitForStart();

        int rings = detector.getDecision();

        if (rings == 4);
        /*
        1. Robot deposits first wobble goal into position C.
        2. Robot trails behind white line, aims, and shoots for three power-shots.
        3. 180 deg turn, collects 2-3 rings, and aims and shoots for top goal.
        4. Parks on white line.
       */

    }
}