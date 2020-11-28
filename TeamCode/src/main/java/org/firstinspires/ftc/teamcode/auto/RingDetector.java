package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class RingDetector {
    OpMode opMode;
    OpenCvCamera camera;

    CustomPipeline pipeline;

    private final Point BLUE_TOP_TL = new Point(90,90);
    private final Point BLUE_TOP_BR = new Point(140, 120);
    private final Point BLUE_BOTTOM_TL = new Point(185, 90);
    private final Point BLUE_BOTTOM_BR = new Point(235,  120);

    private final Point RED_TOP_TL = new Point(0,135);
    private final Point RED_TOP_BR = new Point(50, 165);
    private final Point RED_BOTTOM_TL = new Point(62, 140);
    private final Point RED_BOTTOM_BR = new Point(125,  170);

    private Point topTL;
    private Point topBR;
    private Point bottomTL;
    private Point bottomBR;

    private RGBColor top;
    private RGBColor bottom;

    public RingDetector(OpMode op, boolean isRed){

        opMode = op;

        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

        pipeline = new CustomPipeline();
        camera.openCameraDevice();
        camera.setPipeline(pipeline);
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        topTL = (isRed) ? RED_TOP_TL : BLUE_TOP_TL;
        topBR = (isRed) ? RED_TOP_BR : BLUE_TOP_BR;
        bottomTL = (isRed) ? RED_BOTTOM_TL : BLUE_BOTTOM_TL;
        bottomBR = (isRed) ? RED_BOTTOM_BR : BLUE_BOTTOM_BR;
    }

    public void stopStreaming(){
        camera.stopStreaming();
    }

    public int getDecision(){
        int topValue = top.getValue();
        int bottomValue = bottom.getValue();
        opMode.telemetry.addData("Top Value: ", topValue);
        opMode.telemetry.addData("Bottom Value: ", bottomValue);
        opMode.telemetry.update();
        boolean topRing = false;
        boolean bottomRing = false;
        if (topValue > 70) {
            topRing = true;
        }
        if (bottomValue > 70) {
            bottomRing = true;
        }
        if (topRing && bottomRing)
            return 4;
        else if (bottomRing)
            return 1;
        else return 0;
    }

    class CustomPipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input){

            top = getAverageColor(input, topTL, topBR);
            bottom = getAverageColor(input, bottomTL, bottomBR);

            int thickness = 4;
            Scalar topColor = new Scalar(255,0,0);
            Scalar bottomColor = new Scalar(255,0,0);
            int position = getDecision();
            if (position == 4){
                topColor = new Scalar(0,255,0);
                bottomColor = new Scalar(0,255,0);
            }
            else if (position == 1){
                bottomColor = new Scalar(0,255,0);
            }

            Imgproc.rectangle(input, topTL, topBR, topColor, thickness);
            Imgproc.rectangle(input, bottomTL, bottomBR, bottomColor, thickness);

            //sendTelemetry();

            return input;
        }

        private RGBColor getAverageColor(Mat mat, Point topLeft, Point bottomRight){
            int red = 0;
            int green = 0;
            int blue = 0;
            int total = 0;

            for (int x = (int)topLeft.x; x < bottomRight.x; x++){
                for (int y = (int)topLeft.y; y < bottomRight.y; y++){
                    red += mat.get(y,x)[0];
                    green += mat.get(y,x)[1];
                    blue += mat.get(y,x)[2];
                    total++;
                }
            }

            red /= total;
            green /= total;
            blue /= total;

            return new RGBColor(red, green, blue);
        }

        private void sendTelemetry(){
            opMode.telemetry.addLine("Top :" + " R " + top.getRed() + " G " + top.getGreen() + " B " + top.getBlue());
            opMode.telemetry.addLine("Bottom :" + " R " + bottom.getRed() + " G " + bottom.getGreen() + " B " + bottom.getBlue());
            opMode.telemetry.update();
        }

    }
}
