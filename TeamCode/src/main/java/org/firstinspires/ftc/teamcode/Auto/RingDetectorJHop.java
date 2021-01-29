package org.firstinspires.ftc.teamcode.Auto;

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

/*
public class RingDetectorJHop {
    OpMode opMode;
    OpenCvCamera camera;

    CustomPipeline pipeline;

    private final Point JHOP_TL    = new Point(120,40);
    private final Point JHOP_BR    = new Point(190, 70);

    private Point TL;
    private Point BR;

    private RGBColor box;


    public RingDetectorJHop(OpMode op){

        opMode = op;

        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, "J-Hop Webcam"), cameraMonitorViewId);

        pipeline = new CustomPipeline();
        camera.openCameraDevice();
        camera.setPipeline(pipeline);
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        TL = JHOP_TL;
        BR = JHOP_BR;
    }

    public void stopStreaming(){
        camera.stopStreaming();
    }

    public int getDecision(){
        int boxValue = box.getValue();
        opMode.telemetry.addData("Value: ", boxValue);
        opMode.telemetry.update();

        if (boxValue < 200 ) {
            return 1;
        } else if (boxValue < 280 /){
            return 2;
        } else if (boxValue < 300 ) {
            return 3;
        } else {
            return 0;
        }
    }

    class CustomPipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input){

            box = getAverageColor(input, TL, BR);

            int thickness = 3;
            Scalar boxColor = new Scalar(255,0,0);
                //removed bottomColor
            int position = getDecision();
            if (position == 0){
                boxColor = new Scalar(0,255,0);
            }
            else if (position == 1 || position == 2 || position == 3){
                boxColor = new Scalar(255,0,0);
            }

            Imgproc.rectangle(input, TL, BR, boxColor, thickness);

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
            opMode.telemetry.addLine("Box Values :" + " R " + box.getRed() + " G " + box.getGreen() + " B " + box.getBlue());
        }

    }
}
*/