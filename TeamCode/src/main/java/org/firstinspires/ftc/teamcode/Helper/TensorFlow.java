package org.firstinspires.ftc.teamcode.Helper;

import android.os.SystemClock;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import java.util.List;

@Config
public class TensorFlow {
    public static class Params {
        public boolean OVERRIDE_TFOD_DETECTION = false;
        public int OVERRIDE_SPIKEMARK_POS = 2;

        public int cameraStreamingWait = 1000;
        public String tfodModelFile = "/sdcard/FIRST/tflitemodels/model_Training5.tflite";
        public double tfodMinConfidence = 0.90;
        public double spikemarkPositionOneMax_X = 160;
        public double propMinWidth = 80;
        public double propMaxWidth = 200;
    }

    public static  Params PARAMS = new Params();

    private static final String[] LABELS = { "Prop", };

    private final TfodProcessor tfod;
    private final VisionPortal visionPortal;

    public int tlmObjectCnt = 0;
    public int tlmPropCnt  = 0;
    public double tlmBestPropArea = 0;
    public double tlmBestPropXPos = 0;
    public double tlmBestPropYPos = 0;
    public double tlmConfidence = 0;


    public TensorFlow (@NonNull  HardwareMap hdwMap) {
        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                .setModelFileName(PARAMS.tfodModelFile)
                .setModelLabels(LABELS)
                .build();

        // Set confidence threshold for TFOD recognitions, can change at any time.
        tfod.setMinResultConfidence((float) PARAMS.tfodMinConfidence);

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tfod)
                .setCamera(hdwMap.get(WebcamName.class, "Webcam Front"))
                .build();

        visionPortal.setProcessorEnabled(tfod, true);
    }


    public boolean isCameraStreaming() {
        boolean cameraStreaming = (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING);
        return cameraStreaming;
    }


    public int DetectProp() {
        // Check for Tfod Override and Exit if true
        if (PARAMS.OVERRIDE_TFOD_DETECTION)
            return PARAMS.OVERRIDE_SPIKEMARK_POS;

        // while no object and not timed out
        tlmObjectCnt = 0;
        tlmPropCnt = 0;
        tlmBestPropArea = 0;
        tlmBestPropXPos = 0;
        tlmBestPropYPos = 0;

        // Step through the list of recognitions and display info for each one.
        List<Recognition> currentRecognitions = tfod.getRecognitions();

        for (Recognition recognition : currentRecognitions) {
            ++tlmObjectCnt;
            double x = (recognition.getLeft() + recognition.getRight()) / 2; //getting the coordinates for the box --> finding the middle of the box
            double y = (recognition.getTop() + recognition.getBottom()) / 2;
            double height = recognition.getHeight();
            double width = recognition.getWidth();

            // Check for Shape Parameters
            if ((width >= PARAMS.propMinWidth) && (width <= PARAMS.propMaxWidth)) {
                ++tlmPropCnt;
                if ((height * width) > tlmBestPropArea) {
                    tlmBestPropArea = height * width;
                    tlmBestPropXPos = x;
                    tlmBestPropYPos = y;
                    tlmConfidence = recognition.getConfidence();
                }
            }
        }

        int propNum = 3;
        if (tlmPropCnt > 0) {
            if (tlmBestPropXPos <= PARAMS.spikemarkPositionOneMax_X)
                propNum = 1;
            else
                propNum = 2;
        }

        return propNum;
    }

    public void CleanUp () {
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopStreaming();
        }
    }
}
