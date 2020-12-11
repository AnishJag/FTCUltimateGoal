package org.firstinspires.ftc.teamcode.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="BlueCaseC")
public class BlueCaseC extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RingDetector detector = new RingDetector(this, false);
        int rings = detector.getDecision();
        waitForStart();

        if (rings == 4);
        /*
        1. Robot deposits first wobble goal into position C.
        2. Robot trails behind white line, aims, and shoots for three power-shots.
        3. 180 deg turn, collects 2-3 rings, and aims and shoots for top goal.
        4. Parks on white line.
       */

    }
}