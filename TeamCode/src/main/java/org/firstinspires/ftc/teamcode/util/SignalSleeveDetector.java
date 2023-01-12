package org.firstinspires.ftc.teamcode.util;

import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.vision.CVUtility;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.outoftheboxrobotics.tensorflowapi.ImageClassification.TFICBuilder;
import org.outoftheboxrobotics.tensorflowapi.ImageClassification.TensorImageClassifier;

import java.io.IOException;
import java.util.List;

public class SignalSleeveDetector {

    private static TensorImageClassifier classifier;
    public static boolean started = false;

    public static void initializeTensorFlow(HardwareManager manager, Telemetry telem, CVUtility cv) throws IOException {
        while (!cv.initialized)
        {
            telem.addData("Initializing", "");
            telem.update();
        }

        telem.clearAll();

        classifier = new TFICBuilder(manager.hardwareMap, "bulldogsVision.tflite", "Dot 1", "Dot 2", "Dot 3")
                .useXNNPack(true)
                .build();

//        try {
//            VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters();
//
//            params.vuforiaLicenseKey = RobotConstants.vuforiaKey;
//            params.cameraName = manager.hardwareMap.get(WebcamName.class, "Webcam 1");
//
//            vuforia = ClassFactory.getInstance().createVuforia(params);
//
//            TFObjectDetector.Parameters tfParams = new TFObjectDetector.Parameters();
//
//            tfParams.minResultConfidence = 0.6f;
//            tfParams.isModelTensorFlow2 = true;
//            tfParams.inputSize = 300;
//
//            tfod = ClassFactory.getInstance().createTFObjectDetector(tfParams, vuforia);
//
//            tfod.loadModelFromFile(RobotConstants.asset);
//        } catch (Exception e)
//        {
//            tfod = null;
//        }
//
//        if (tfod != null) {
//            tfod.activate();
//
//            tfod.setZoom(1.0, 16.0/9.0);
//
//            telem.addData("Ready to start.", "");
//            telem.update();
//        }
    }

    public static Zone detectZone(CVUtility cv, Telemetry telem)
    {
        assert classifier != null;

        try {
            if (classifier == null)
                return Zone.ZONE_TWO;

            float confidence = 0;

            TensorImageClassifier.Recognition zone = null;

            List<TensorImageClassifier.Recognition> recognitions = classifier.recognize(cv.grabFrame());

            for (TensorImageClassifier.Recognition recognition : recognitions)
            {

                if (zone == null)
                    zone = recognition;
                else if (recognition.getConfidence() > zone.getConfidence())
                    zone = recognition;
            }

            if (zone == null)
                return Zone.ZONE_TWO;

            telem.addData("Detection success", zone.getTitle() );
            telem.update();

            switch (zone.getTitle())
            {
                case "Dot 1":
                    return Zone.ZONE_ONE;
                case "Dot 2":
                    return Zone.ZONE_TWO;
                case "Dot 3":
                    return Zone.ZONE_THREE;
                default:
                    return Zone.ZONE_TWO;
            }

        } catch (Exception e)
        {
            telem.addData("Detection failed.", "Defaulting to Zone 2");
            telem.update();
            return Zone.ZONE_TWO;
        }
    }

    public static int detectOrientation(Mat mat) {
        if (mat == null) {
            return 1;
        }

        // Do color detection
        Mat hsvMat = new Mat();
        Imgproc.cvtColor(mat, hsvMat, Imgproc.COLOR_RGB2HSV);
        Mat green = new Mat();
        Core.inRange(hsvMat, new Scalar(35, 40, 35), new Scalar(70, 120, 255), green);
        int greenIndex = 0;
        for (int i = green.rows() / 3; i < green.rows() * 2 / 3; i += 5) {
            for (int j = green.cols() / 3; j < green.cols() * 2 / 3; j += 5) {
                if (green.get(i,j)[0] != 0) {
                    greenIndex++;
                }
            }
        }
        Mat blue = new Mat();
        Core.inRange(hsvMat, new Scalar(75, 80, 35), new Scalar(130, 170, 255), blue);
        int blueIndex = 0;
        for (int i = blue.rows() / 3; i < blue.rows() * 2 / 3; i += 5) {
            for (int j = blue.cols() / 3; j < blue.cols() * 2 / 3; j += 5) {
                if (blue.get(i,j)[0] != 0) {
                    blueIndex++;
                }
            }
        }

        int threshold = 200;
        OpModeHolder.opMode.telemetry.addData("blueIndex", blueIndex);
        OpModeHolder.opMode.telemetry.addData("greenIndex", greenIndex);

//        OpModeHolder.opMode.telemetry.update();
        if (greenIndex > threshold || blueIndex > threshold) {
            if (greenIndex > blueIndex) {
                return 2;
            }
            return 3;
        }
        return 1;
    }

    public enum Zone
    {
        ZONE_ONE,
        ZONE_TWO,
        ZONE_THREE
    }

}