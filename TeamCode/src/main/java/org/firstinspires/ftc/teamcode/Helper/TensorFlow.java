package org.firstinspires.ftc.teamcode.Helper;

import static android.os.SystemClock.sleep;

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
        public int OVERIDE_SPIKEMARK_POS = 1;

        public int cameraStreamingWait = 1000;
        public String tfodModelFile = "/sdcard/FIRST/tflitemodels/model_Training2.tflite";
        public double tfodMinConfidence = 0.80;
        public double spikemarkPositionOneMax_X = 170;
        public double propMinWidth = 95;
        public double propMaxWidth = 180;
    }

    public static  Params PARAMS = new Params();

    private static final String[] LABELS = { "Prop", };

    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    public int tlmObjectCnt = 0;
    public int tlmPropCnt  = 0;
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

        boolean cameraStreaming = false;
        long startCameraWait = System.currentTimeMillis();
        boolean timedOut = false;

        while (!cameraStreaming && !timedOut)  {
            cameraStreaming = (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING);
            timedOut = (System.currentTimeMillis() - (startCameraWait + PARAMS.cameraStreamingWait)) > 0;
            SystemClock.sleep(50);
        }
    }

    public int DetectProp(long waitMs) {
        double largestPropArea = 0;
        double largestX = 0;
        double largestY = 0;
        int propNum = 0;

        // Check for Tfod Override and Exit if true
        if (PARAMS.OVERRIDE_TFOD_DETECTION)
            return PARAMS.OVERIDE_SPIKEMARK_POS;

        // while no object and not timed out
        long waitEndTime = (System.currentTimeMillis() + waitMs);
        boolean timeExpired = false;
        tlmObjectCnt = 0;

        // Step through the list of recognitions and display info for each one.
        while (!timeExpired && (largestPropArea == 0)) {
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
                    if ((height * width) > largestPropArea) {
                        largestPropArea = height * width;
                        largestX = x;
                        largestY = y;
                        tlmBestPropXPos = x;
                        tlmBestPropYPos = y;
                        tlmConfidence = recognition.getConfidence();
                    }
                }
            }

            timeExpired = (System.currentTimeMillis() > waitEndTime);
            if (!timeExpired && (largestPropArea == 0))
                sleep( 100);
        }

        propNum = 3;
        if (largestPropArea > 0) {
            if (largestX <= PARAMS.spikemarkPositionOneMax_X)
                propNum = 1;
            else
                propNum = 2;
        }

        return propNum;
    }

    public void CleanUp () {
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING)
            visionPortal.stopStreaming();
        visionPortal.close();
    }
}
