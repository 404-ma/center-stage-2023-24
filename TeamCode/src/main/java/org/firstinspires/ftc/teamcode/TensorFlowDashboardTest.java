package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

@Config
@TeleOp(name="TensorFlow Dashboard Test", group = "Diag")
public class TensorFlowDashboardTest extends LinearOpMode {
    // FTC Dashboard Parameters
    public static class Params {
        // Camera Name in Robot Config
        public String cameraName = "Webcam Front";

        // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
        // this is used when uploading models directly to the RC using the model upload interface.
        public String tfodModelFile = "/sdcard/FIRST/tflitemodels/model_TF_Training20240215.tflite";
    }

    public Params PARAMS = new Params();


    // FTC Dashboard Vision Portal Streaming
    private static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {
        private final AtomicReference<Bitmap> lastFrame =
                new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(frame, b);
            lastFrame.set(b);
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                Object userContext) {
            // do nothing
        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }
    }

    private final CameraStreamProcessor streamProc = new CameraStreamProcessor();


    // Internal Variables
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    private FtcDashboard dashboard;
    private static final String[] LABELS = { "Prop", };  // Labels in the TFOD model (must be in training order!)


    @Override
    public void runOpMode() {
        initialize();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetryTfod();

            // Save CPU resources; can resume streaming when needed.
            if (gamepad1.dpad_down) {
                visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                visionPortal.resumeStreaming();
            }

            // Share the CPU.
            sleep(20);
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();
    }


    // Initialize the TensorFlow Object Detection processor.
    private void initialize() {
        // Create the TensorFlow processor the easy way.
        tfod = new TfodProcessor.Builder()
                .setModelFileName(PARAMS.tfodModelFile)
                .setModelLabels(LABELS)
                .build();

        // Set confidence threshold for TFOD recognitions, can change at any time.
        tfod.setMinResultConfidence(0.80f);

        // Create the vision portal with b.
        visionPortal = new VisionPortal.Builder()
                .addProcessor(tfod)
                .addProcessor(streamProc)
                .setCamera(hardwareMap.get(WebcamName.class, PARAMS.cameraName))
                .build();

        visionPortal.setProcessorEnabled(tfod, true);

        dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(streamProc, 0);
        dashboard.clearTelemetry();
    }


    //  Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
    private void telemetryTfod() {
        if (visionPortal.getProcessorEnabled(tfod)) {
            telemetry.addLine("Dpad Down to disable TFOD");
            telemetry.addLine();
        } else {
            telemetry.addLine("Dpad Up to enable TFOD");
        }

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        int objCnt = 0;

        double largestObjectArea = 0;
        double largestObjectX = 0;
        double largestObjectY = 0;

        for (Recognition recognition : currentRecognitions) {
            ++objCnt;

            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            double height = recognition.getHeight();
            double width = recognition.getWidth();

            if ((height * width) > largestObjectArea) {
                largestObjectArea = height * width;
                largestObjectX = x;
                largestObjectY = y;
            }

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }

        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Objects", objCnt);
        packet.put("Best Obj X", largestObjectX);
        packet.put("Best Obj Y", largestObjectY);
        dashboard.sendTelemetryPacket(packet);
    }
}
