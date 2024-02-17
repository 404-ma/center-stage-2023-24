package org.firstinspires.ftc.teamcode.Helper;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import java.util.List;

public class TensorFlow {
    public static class Params{
        public double propSpikeMark = 2;
    }

    public static Params PARAMS = new Params();
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/model_Training2.tflite";
    private static final String[] LABELS = {
            "Prop",
    };
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    List<Recognition> currentRecognitions = tfod.getRecognitions();


    public void InitTFOD(HardwareMap hdwMap) {
        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                .setModelFileName(TFOD_MODEL_FILE)
                .setModelLabels(LABELS)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hdwMap.get(WebcamName.class, "Webcam Front"));
        builder.addProcessor(tfod);
        visionPortal = builder.build();
    }

    public void telTFOD(){
        telemetry.addData("# Objects Detected", currentRecognitions.size());
        // Step through the list of recognitions and display info for each one.
        int obj =0;
        double largestObj =0;
        double largestX = 0;
        double largestY = 0;

        for (Recognition recognition : currentRecognitions) {
            obj++;
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ; //getting the corrdinates for the box --> finding the middle of the box
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            double height = recognition.getHeight();
            double width = recognition.getWidth();

            if((height * width) > largestObj ){

                largestObj = height * width;
                largestX = x;
                largestY = y;
            }

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", largestX, largestY);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

        telemetry.update();

        if(110<= largestX && largestX <= 140){
            PARAMS.propSpikeMark = 3;
        }
        else if (315<= largestX && largestX <= 590) {
            PARAMS.propSpikeMark = 1;
        }
        else{
            PARAMS.propSpikeMark = 2;
        }


    }






}
