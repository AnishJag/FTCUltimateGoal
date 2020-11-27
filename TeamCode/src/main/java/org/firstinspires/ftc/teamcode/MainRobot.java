package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MainRobot {

    public DcMotor topLeft = null;
    public DcMotor bottomLeft = null;
    public DcMotor topRight = null;
    public DcMotor bottomRight = null;
    public DcMotor jHopper1 = null;
    public DcMotor jHopper2 = null;
    public DcMotor wobbleArm = null;
    public Servo leftClaw = null;

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

    }
}