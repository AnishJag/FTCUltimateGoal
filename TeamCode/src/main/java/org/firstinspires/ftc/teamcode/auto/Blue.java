package org.firstinspires.ftc.teamcode.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Blue")
public class Blue extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RingDetector detector = new RingDetector(this, false);
        int rings = detector.getDecision();
        waitForStart();

        if (rings == 0);


    }
}
