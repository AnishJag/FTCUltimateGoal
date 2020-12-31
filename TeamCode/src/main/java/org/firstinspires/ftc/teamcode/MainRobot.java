package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;

public class MainRobot {

    // Total Motors: 8
    // Total Servos: 0
    public DcMotor topLeft = null;
    public DcMotor bottomLeft = null;
    public DcMotor topRight = null;
    public DcMotor bottomRight = null;
    public DcMotor jHopper1 = null;
    public DcMotor jHopper2 = null;
    public DcMotor wobbleArm = null;
    public DcMotor foamWheel = null;
    // public Servo leftClaw = null;

    // Total Sensors: 4
    public ModernRoboticsI2cRangeSensor frontRange  = null;
    public ModernRoboticsI2cRangeSensor leftRange   = null;
    public ModernRoboticsI2cRangeSensor rightRange  = null;
    public ModernRoboticsI2cGyro        gyro        = null;

    static final double     COUNTS_PER_MOTOR_REV    = 386.3;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;
    static final double     WHEEL_DIAMETER_INCHES   = 3.77953;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
                                                             / (WHEEL_DIAMETER_INCHES * 3.14159265);

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
}