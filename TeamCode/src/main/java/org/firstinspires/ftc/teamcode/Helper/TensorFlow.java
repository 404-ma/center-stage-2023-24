package org.firstinspires.ftc.teamcode.Helper;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import java.util.List;

public class TensorFlow {
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/model_Training2.tflite";
    private static final String[] LABELS = { "Prop", };
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    public int tlmObjectCnt = 0;
    public double tlmBestObjectX = 0;
    public double tlmBestObjectY = 0;
    public double tlmConfidence = 0;


    public TensorFlow (HardwareMap hdwMap) {
        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                .setModelFileName(TFOD_MODEL_FILE)
                .setModelLabels(LABELS)
                .build();

        // Set confidence threshold for TFOD recognitions, can change at any time.
        tfod.setMinResultConfidence(0.80f);

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tfod)
                .setCamera(hdwMap.get(WebcamName.class, "Webcam Front"))
                .build();

        visionPortal.setProcessorEnabled(tfod, true);
    }

    public int telemTFOD(long waitMs) {
        double largestObj = 0;
        double largestX = 0;
        double largestY = 0;
        int propNum = 0;

        // while no object and not timed out
        long waitEndTime = (System.currentTimeMillis() + waitMs);
        tlmObjectCnt = 0;

        // Step through the list of recognitions and display info for each one.
        while ((waitEndTime > System.currentTimeMillis()) && (largestObj == 0)) {
            List<Recognition> currentRecognitions = tfod.getRecognitions();

            for (Recognition recognition : currentRecognitions) {
                ++tlmObjectCnt;
                double x = (recognition.getLeft() + recognition.getRight()) / 2; //getting the coordinates for the box --> finding the middle of the box
                double y = (recognition.getTop() + recognition.getBottom()) / 2;
                double height = recognition.getHeight();
                double width = recognition.getWidth();

                if ((height * width) > largestObj) {
                    largestObj = height * width;
                    largestX = x;
                    largestY = y;
                    tlmBestObjectX = x;
                    tlmBestObjectY = y;
                    tlmConfidence = recognition.getConfidence();
                }
            }

            if (largestObj == 0) sleep( 100);
        }

        if (largestObj > 0) {
            if (largestX <= 180)
                propNum = 1;
            else
                propNum = 2;
        } else
            propNum = 3;

        return propNum;
    }

    public void CleanUp () {
        visionPortal.close();
    }
}
