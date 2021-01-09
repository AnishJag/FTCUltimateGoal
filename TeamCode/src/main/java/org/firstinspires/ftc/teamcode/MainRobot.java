package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;

public class MainRobot {

    // Total Motors: 8
    // Total Servos: 1
    public DcMotor topLeft = null;
    public DcMotor bottomLeft = null;
    public DcMotor topRight = null;
    public DcMotor bottomRight = null;
    public DcMotor jHopper1 = null;
    public DcMotor jHopper2 = null;
    public DcMotor wobbleArm = null;
    public DcMotor foamWheel = null;
    public CRServo wobbleClaw = null;

    // Total Sensors: 4
    public ModernRoboticsI2cRangeSensor frontRange  = null;
    public ModernRoboticsI2cRangeSensor leftRange   = null;
    public ModernRoboticsI2cRangeSensor rightRange  = null;
    public ModernRoboticsI2cGyro        gyro        = null;

    static final double     COUNTS_PER_MOTOR_REV    = 386.3;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;
    static final double     WHEEL_DIAMETER_INCHES   = 3.77953;
    public static final double     COUNTS_PER_INCH  = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
                                                             / (WHEEL_DIAMETER_INCHES * 3.14159265);
    public static final double     DRIVE_SPEED        = 1.0;
    public static final double     TURN_SPEED         = 0.5;
    public static final double     P_TURN_COEFF       = 0.1;
    public static final double     HEADING_THRESHOLD  = 1.0;

    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        topLeft = hwMap.get(DcMotor.class, "topLeft");
        bottomLeft = hwMap.get(DcMotor.class, "bottomLeft");
        topRight = hwMap.get(DcMotor.class, "topRight");
        bottomRight = hwMap.get(DcMotor.class, "bottomRight");
        jHopper1 = hwMap.get(DcMotor.class, "jHopper1");
        jHopper2 = hwMap.get(DcMotor.class, "jHopper2");
        wobbleArm = hwMap.get(DcMotor.class, "wobbleArm");
        foamWheel = hwMap.get(DcMotor.class, "foamWheel");
        wobbleClaw = hwMap.get(CRServo.class,"wobbleClaw");

        topLeft.setDirection(DcMotor.Direction.REVERSE);
        bottomLeft.setDirection(DcMotor.Direction.REVERSE);
        topRight.setDirection(DcMotor.Direction.FORWARD);
        bottomRight.setDirection(DcMotor.Direction.FORWARD);

        topLeft.setPower(0);
        bottomLeft.setPower(0);
        topRight.setPower(0);
        bottomRight.setPower(0);
        jHopper1.setPower(0);
        jHopper2.setPower(0);
        wobbleArm.setPower(0);
        foamWheel.setPower(0);
        wobbleClaw.setPower(0);


        frontRange = hwMap.get(ModernRoboticsI2cRangeSensor.class,"frontRange");
        frontRange.initialize();

        leftRange  = hwMap.get(ModernRoboticsI2cRangeSensor.class,"leftRange");
        leftRange.initialize();

        rightRange  = hwMap.get(ModernRoboticsI2cRangeSensor.class,"rightRange");
        rightRange.initialize();

        gyro = hwMap.get(ModernRoboticsI2cGyro.class,"gyro");
        gyro.initialize();
        gyro.calibrate();
    }

    public double getError (double angle){

        double robotError;

        // Calculates error from angle.
        robotError = angle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer (double error, double P_DRIVE_COEFF){
        return Range.clip(error * P_DRIVE_COEFF, -1, 1);
    }

    //Autonomous Blue & Red
    public void gyroDrive (double speed, double distanceTL, double distanceTR,
                           double distanceBL, double distanceBR, double angle, LinearOpMode opmode) {

        int     newTLTarget;
        int     newTRTarget;
        int     newBLTarget;
        int     newBRTarget;
        int     moveCountsTL = (int)(distanceTL * MainRobot.COUNTS_PER_INCH);
        int     moveCountsTR = (int)(distanceTR * MainRobot.COUNTS_PER_INCH);
        int     moveCountsBL = (int)(distanceBL * MainRobot.COUNTS_PER_INCH);
        int     moveCountsBR = (int)(distanceBR * MainRobot.COUNTS_PER_INCH);
        double  max;
        double  leftMax;
        double  rightMax;
        double  error;
        double  speedTL;
        double  speedTR;
        double  speedBL;
        double  speedBR;

        // Ensure that the Op-mode is still active
        if (opmode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTLTarget  = topLeft.getCurrentPosition() + moveCountsTL;
            newTRTarget  = topRight.getCurrentPosition() + moveCountsTR;
            newBLTarget  = bottomLeft.getCurrentPosition() + moveCountsBL;
            newBRTarget  = bottomRight.getCurrentPosition() + moveCountsBR;

            // Set Target and Turn On RUN_TO_POSITION
            topLeft.setTargetPosition(newTLTarget);
            topRight.setTargetPosition(newTRTarget);
            bottomLeft.setTargetPosition(newBLTarget);
            bottomRight.setTargetPosition(newBRTarget);

            topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            topLeft.setPower(speed);
            topRight.setPower(speed);
            bottomLeft.setPower(speed);
            bottomRight.setPower(speed);

            // keep looping while we are still active, and ALL motors are running.
            while (opmode.opModeIsActive() &&
                    (topLeft.isBusy() && topRight.isBusy() && bottomLeft.isBusy() && bottomRight.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                double steer = getSteer(error, 0.15);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distanceTL > 0)
                    speedTL  = speed - steer;
                else speedTL = speed + steer;

                if (distanceTR > 0)
                    speedTR  = speed + steer;
                else speedTR = speed - steer;

                if (distanceBL > 0)
                    speedBL  = speed - steer;
                else speedBL = speed + steer;

                if (distanceBR > 0)
                    speedBR  = speed + steer;
                else speedBR = speed - steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                leftMax  = Math.max(Math.abs(speedTL), Math.abs(speedTR));
                rightMax = Math.max(Math.abs(speedBL), Math.abs(speedBR));
                max      = Math.max(leftMax,rightMax);
                if (max > 1.0)
                {
                    speedTL /= max;
                    speedTR /= max;
                    speedBL /= max;
                    speedBR /= max;
                }

                topLeft.setPower(speedTL);
                topRight.setPower(speedTR);
                bottomLeft.setPower(speedBL);
                bottomRight.setPower(speedBR);

                // Display drive status for the driver.
                opmode.telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                opmode.telemetry.addData("Target",  "%7d:%7d:%7d:%7d", newTLTarget, newTRTarget, newBLTarget, newBRTarget);
                opmode.telemetry.addData("Actual",  "%7d:%7d:%7d:%7d", topLeft.getCurrentPosition(),
                        topRight.getCurrentPosition(), bottomLeft.getCurrentPosition(), bottomRight.getCurrentPosition() );
                opmode.telemetry.addData("Speed",   "%5.2f:%5.2f:%5.2f:%5.2f", speedTL, speedTR, speedBL, speedBR);
                opmode.telemetry.update();
            }

            // Stop all motion;
            topLeft.setPower(0);
            topRight.setPower(0);
            bottomLeft.setPower(0);
            bottomRight.setPower(0);

            // Turn off RUN_TO_POSITION
            topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void gyroTurn (double speed, double angle, LinearOpMode opmode) {


        // keep looping while we are still active, and not on heading.
        while (opmode.opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF, opmode)) {
            // Update telemetry & Allow time for other processes to run.
            opmode.telemetry.update();
        }
    }

    boolean onHeading(double speed, double angle, double PCoeff, LinearOpMode opmode) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        topLeft.setPower(leftSpeed);
        topRight.setPower(rightSpeed);
        bottomLeft.setPower(leftSpeed);
        bottomRight.setPower(rightSpeed);

        // Display it for the driver.
        opmode.telemetry.addData("Target", "%5.2f", angle);
        opmode.telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        opmode.telemetry.addData("Speed ", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }
}