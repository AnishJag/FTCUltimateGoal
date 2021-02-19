package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Red")
@Disabled
public class Red extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RingDetector detector = new RingDetector(this, true);

        waitForStart();

        int rings = detector.getDecision();

        if (rings == 0);
        /*
        1. Robot deposits first wobble goal into position A.
        2. Trails back to pick up second wobble goal.
        3. Robot deposits second wobble goal into position A.
        4. Robot trails behind white line, aims, and shoots for three power-shots.
        5. After completion of steps 1-4, robot parks on white line.
        This process is the same for if (rings == 1).
        */
    }
}
