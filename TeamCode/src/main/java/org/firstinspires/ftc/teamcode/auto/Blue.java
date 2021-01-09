package org.firstinspires.ftc.teamcode.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MainRobot;


@Autonomous(name="Blue")
public class Blue extends LinearOpMode {

    MainRobot robot = new MainRobot();

    @Override
    public void runOpMode() throws InterruptedException {

        RingDetector detector = new RingDetector(this, false);

        robot.init(hardwareMap);

        robot.topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        robot.gyro.resetZAxisIntegrator();

        int rings = detector.getDecision();

        if (rings == 0){
            robot.gyroDrive(robot.DRIVE_SPEED,3,3,3,3,0,this);
            /*robot.gyroTurn(robot.TURN_SPEED,3,this);*/
            robot.wobbleArm.setPower(1);
            sleep(1000);
            robot.wobbleArm.setPower(-1);
            sleep(1000);
            robot.wobbleArm.setPower(0);
        }
        else if(rings == 1){
            robot.gyroDrive(robot.DRIVE_SPEED,3,3,3,3,0,this);
        }
        /*
        1. Robot deposits first wobble goal into position A.
        2. Trails back to pick up second wobble goal.
        3. Robot deposits second wobble goal into position A.
        4. Robot trails behind white line, aims, and shoots for three power-shots.
        5. After completion of steps 1-4, robot parks on white line.
        This process is the same for "if (rings == 1).
       */
    }

}
