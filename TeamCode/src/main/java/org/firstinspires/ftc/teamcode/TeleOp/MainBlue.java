package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.Auto.RingDetectorJHop;
import org.firstinspires.ftc.teamcode.Auto.RingDetectorJHop;
import org.firstinspires.ftc.teamcode.MainRobot;

@TeleOp(name="Main BLUE")
public class MainBlue extends LinearOpMode {

    MainRobot robot = null;

    public static final double FRONT_ALIGNMENT = 67;
    public static final double  LEFT_ALIGNMENT = 42;
    public static final double PWR_FRONT_ALIGNMENT = 55.9;
    public static final double PWR_LEFT_ALIGNMENT = 66.9;
    public static final double PWR_LEFT_SHIFT = 7;
    public static final double MARGIN_OF_ERROR = 1;
    public static final double SLOWDOWN_DISTANCE = 5;
    public static final double SLOWDOWN_POWER  = 0.5;
    public boolean                 GP1_RB_Held = false;
    public boolean                    SlowMode = false;
    public boolean               FieldRelative = true;
    public boolean                 GP1_LB_Held = false;
    public boolean                GP2_DPL_Held = false;
    public boolean                 GP2_RT_Held = false;
    public boolean                  GP1_X_Held = false;
    public double                    FLAP_OPEN = 0.5;
    public double                  FLAP_CLOSED = 0.1;
    public double                    speedJHop = -0.95;

    //RingDetectorJHop detectorJHop = null;
    @Override
    public void runOpMode(){
        custom_init();
        waitForStart();
        while(opModeIsActive()){
            custom_loop();
        }
    }

    public void custom_init() {
        robot = new MainRobot();
        robot.init(hardwareMap);
        //detectorJHop = new RingDetectorJHop(this,false);

        telemetry.addData("Initialization Complete!","");
        telemetry.update();
    }

    public void custom_loop() {
        //int rings = detectorJHop.getDecision();
        //telemetry.addData("Ring Number: ", rings);

        //--------------------DRIVE-TRAIN CONTROLS--------------------\\
        double forward = -gamepad1.left_stick_y;
        double right   =  gamepad1.left_stick_x;

        if(FieldRelative){
            double angle = robot.gyro.getHeading() + 180;
            angle = Math.toRadians(angle);

            double relativeForward = (Math.cos(angle)*right) + (Math.sin(angle)*forward);
            double relativeRight = (Math.sin(angle)*right) - (Math.cos(angle)*forward);

            forward = relativeForward;
            right   = relativeRight;
        }

        double turn = gamepad1.right_stick_x;

        double leftFrontPower = forward + right + turn;
        double leftBackPower = forward - right + turn;
        double rightFrontPower = forward - right - turn;
        double rightBackPower = forward + right - turn;
        double[] powers = {leftFrontPower, leftBackPower, rightFrontPower, rightBackPower};

        boolean needToScale = false;
        for (double power : powers){
            if(Math.abs(power) > 1){
                needToScale = true;
                break;
            }
        }
        if (needToScale){
            double greatest = 0;
            for (double power : powers){
                if (Math.abs(power) > greatest){
                    greatest = Math.abs(power);
                }
            }
            leftFrontPower /= greatest;
            leftBackPower /= greatest;
            rightFrontPower /= greatest;
            rightBackPower /= greatest;
        }


        //--------------------FIELD-RELATIVE DRIVE--------------------\\
        if(gamepad1.left_bumper && !GP1_LB_Held){
            FieldRelative = !FieldRelative;
            GP1_LB_Held = true;
        }
        if(!gamepad1.left_bumper){
            GP1_LB_Held = false;
        }


        //--------------------SLOW-MODE--------------------\\
        if(gamepad1.right_bumper && !GP1_RB_Held){
            GP1_RB_Held = true;
            SlowMode    =  !SlowMode;
        }
        if(!gamepad1.right_bumper){
            GP1_RB_Held = false;
        }

        if(SlowMode) {
            robot.topLeft.setPower(0.3 * leftFrontPower);
            robot.bottomLeft.setPower(0.3 * leftBackPower);
            robot.topRight.setPower(0.3 * rightFrontPower);
            robot.bottomRight.setPower(0.3 * rightBackPower);
        }
        else{
            robot.topLeft.setPower(leftFrontPower);
            robot.bottomLeft.setPower(leftBackPower);
            robot.topRight.setPower(rightFrontPower);
            robot.bottomRight.setPower(rightBackPower);
        }

        //--------------------AUTOMATION CONTROLS--------------------\\

        //---------------TOP-GOAL LINEUP---------------\\
        if(gamepad1.a){
            double frontRange = robot.frontRange.getDistance(DistanceUnit.INCH);
            double leftRange   = robot.leftRange.getDistance(DistanceUnit.INCH);
            telemetry.addData("Front Range: ", frontRange);
            telemetry.addData("Left Range: ", leftRange);

            double forwardPower = 0;
            double rightPower    = 0;

            if (frontRange < FRONT_ALIGNMENT - MARGIN_OF_ERROR){
                if((FRONT_ALIGNMENT - frontRange) <= SLOWDOWN_DISTANCE) {
                    forwardPower = -SLOWDOWN_POWER;
                }
                else{
                    forwardPower = -1;
                }
            }
            else if(frontRange > FRONT_ALIGNMENT + MARGIN_OF_ERROR){
                if((frontRange - FRONT_ALIGNMENT) <= SLOWDOWN_DISTANCE){
                    forwardPower = SLOWDOWN_POWER;
                }
                else {
                    forwardPower = 1;
                }
            }
            if(leftRange < LEFT_ALIGNMENT - MARGIN_OF_ERROR){
                if((LEFT_ALIGNMENT - leftRange) <= SLOWDOWN_DISTANCE){
                    rightPower = SLOWDOWN_POWER;
                }
                else{
                    rightPower = 1;
                }
            }
            else if(leftRange > LEFT_ALIGNMENT + MARGIN_OF_ERROR){
                if((leftRange - LEFT_ALIGNMENT) <= SLOWDOWN_DISTANCE){
                    rightPower = -SLOWDOWN_POWER;
                }
                else {
                    rightPower = -1;
                }
            }
            robot.topLeft.setPower(forwardPower + rightPower);
            robot.topRight.setPower(forwardPower - rightPower);
            robot.bottomLeft.setPower(forwardPower - rightPower);
            robot.bottomRight.setPower(forwardPower + rightPower);
        }

        //---------------POWER-SHOT & SHOOT---------------\\
        if(gamepad1.b){
            double frontRange = robot.frontRange.getDistance(DistanceUnit.INCH);
            double leftRange   = robot.leftRange.getDistance(DistanceUnit.INCH);
            telemetry.addData("Front Range: ", frontRange);
            telemetry.addData("Left Range: ", leftRange);

            double forwardPower = 0;
            double rightPower    = 0;

            if (frontRange < PWR_FRONT_ALIGNMENT - MARGIN_OF_ERROR){
                if((PWR_FRONT_ALIGNMENT - frontRange) <= SLOWDOWN_DISTANCE) {
                    forwardPower = -SLOWDOWN_POWER;
                }
                else{
                    forwardPower = -1;
                }
            }
            else if(frontRange > PWR_FRONT_ALIGNMENT + MARGIN_OF_ERROR){
                if((frontRange - PWR_FRONT_ALIGNMENT) <= SLOWDOWN_DISTANCE){
                     forwardPower = SLOWDOWN_POWER;
                }
                else {
                     forwardPower = 1;
                }
            }
            if(leftRange < PWR_LEFT_ALIGNMENT - MARGIN_OF_ERROR){
                if((PWR_LEFT_ALIGNMENT - leftRange) <= SLOWDOWN_DISTANCE){
                    rightPower = SLOWDOWN_POWER;
                }
                else{
                    rightPower = 1;
                }
            }
            else if(leftRange > PWR_LEFT_ALIGNMENT + MARGIN_OF_ERROR){
                if((leftRange - PWR_LEFT_ALIGNMENT) <= SLOWDOWN_DISTANCE){
                    rightPower = -SLOWDOWN_POWER;
                }
                else {
                    rightPower = -1;
                }
            }
            robot.topLeft.setPower(forwardPower + rightPower);
            robot.topRight.setPower(forwardPower - rightPower);
            robot.bottomLeft.setPower(forwardPower - rightPower);
            robot.bottomRight.setPower(forwardPower + rightPower);
        }

        if(gamepad1.x && !GP1_X_Held){
            GP1_X_Held = true;
            robot.gyroDrive(0.6, -PWR_LEFT_SHIFT, PWR_LEFT_SHIFT, PWR_LEFT_SHIFT, -PWR_LEFT_SHIFT, 0, 0,0,0,0,this);
        }
        if (!gamepad1.x){
            GP1_X_Held = false;
        }

        //--------------------ROBOT CONTROLS--------------------\\

        //---------------FOAM WHEEL---------------\\
        if (gamepad2.a || gamepad2.left_bumper || gamepad2.right_bumper){
            robot.foamWheel.setPower(1);
        }
        else if (gamepad2.dpad_down){
            robot.foamWheel.setPower(-1);
        }
        else{
            robot.foamWheel.setPower(0);
        }

        //---------------J-HOPPER 1---------------\\
        if (gamepad2.b || gamepad2.left_bumper || gamepad2.right_bumper){
            robot.jHopper1.setPower(-1);
        }
        else if (gamepad2.dpad_right){
            robot.jHopper1.setPower(1);
        }
        else{
            robot.jHopper1.setPower(0);
        }

        //---------------J-HOPPER 2---------------\\
        if (gamepad2.y || gamepad2.left_bumper || gamepad2.right_bumper /* || rings > 0*/){
            robot.jHopper2.setPower(speedJHop);
        }
        else if (gamepad2.dpad_up){
            robot.jHopper2.setPower(0.3);
        }
        else{
            robot.jHopper2.setPower(0);
        }
        //SPEED TOGGLING
        if((gamepad2.right_trigger > .5) && !GP2_RT_Held){
            if(speedJHop == -.95){
                speedJHop = -.8;
            }
            else {
                speedJHop = -.95;
            }
            GP2_RT_Held = true;
        }
            if(gamepad2.right_trigger < .5){
                GP2_RT_Held = false;
            }
        //---------------WOBBLE---------------\\
        double wobbleArm = gamepad2.right_stick_y;
        if(Math.abs(wobbleArm) < 0.1){
            robot.wobbleArm.setPower(0);
        }
        else{
            robot.wobbleArm.setPower(wobbleArm);
        }
        if(gamepad2.x){
            robot.wobbleClaw.setPower(1);
        }
        else{
            robot.wobbleClaw.setPower(0);
        }

        //---------------J-HOPPER FLAP---------------\\
        if(gamepad2.dpad_left && !GP2_DPL_Held) {
            GP2_DPL_Held = true;
            if(Math.abs(robot.JHopFlap.getPosition() - FLAP_CLOSED) < 0.01){
                robot.JHopFlap.setPosition(FLAP_OPEN);
            }
            else{
                robot.JHopFlap.setPosition(FLAP_CLOSED);
            }
        }

        if(!gamepad2.dpad_left) {
            GP2_DPL_Held = false;
        }
        telemetry.update();

        if((Math.abs(robot.JHopFlap.getPosition() - FLAP_CLOSED) < 0.01) /* && rings == 0*/){
            robot.JHopFlap.setPosition(FLAP_CLOSED);
        }

        double castedFront   = Math.round((robot.frontRange.getDistance(DistanceUnit.INCH) * 1000)) / 1000.0;
        double castedLeft    = Math.round((robot.leftRange.getDistance(DistanceUnit.INCH) * 1000)) / 1000.0;
        double castedRight   = Math.round((robot.rightRange.getDistance(DistanceUnit.INCH) * 1000)) / 1000.0;
        double castedJHop    = Math.round((robot.jHopper2.getPower() * 1000) / 1000.0);

        telemetry.addData("Front Distance: ", castedFront);
        telemetry.addData("Right Distance: ", castedRight);
        telemetry.addData("Left Distance: ", castedLeft);
        telemetry.addData("J-Hopper 2 Power: ", castedJHop);
        telemetry.addData("Gyro Angle: ", robot.gyro.getIntegratedZValue());
        telemetry.update();
    }
}
