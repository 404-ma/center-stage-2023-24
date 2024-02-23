package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.os.SystemClock;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

@Config
@TeleOp(name="TensorFlow Dashboard Test", group = "Hardware")
public class TensorFlowDashboardTest extends LinearOpMode {
    // FTC Dashboard Parameters
    public static class Params {
        // Camera Name in Robot Config
        public String cameraName = "Webcam Front";
        public int cameraStreamingWait = 1000;
        public boolean cameraManualExposure = false;
        public long cameraExposureMS = 6;
        public int cameraExposureGain = 250;

        // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
        // this is used when uploading models directly to the RC using the model upload interface.
        //public String tfodModelFile = "/sdcard/FIRST/tflitemodels/model_TF_Training20240215.tflite";
        public String tfodModelFile = "/sdcard/FIRST/tflitemodels/model_Training2.tflite";
    }

    public static Params PARAMS = new Params();


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
        boolean good_init = initialize();
        waitForStart();
        if (isStopRequested() || !good_init)
            return;
        telemetry.clear();

        while (opModeIsActive()) {
            telemetryTfod();

            // Save CPU resources; can resume streaming when needed.
            if (gamepad1.dpad_down) {
                visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                visionPortal.resumeStreaming();
            }

            // Share the CPU.
            sleep(200);
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();
    }


    // Initialize the TensorFlow Object Detection processor.
    private boolean initialize() {
        boolean success = true;

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.addLine("TensorFlow FTC Dashboard Test");
        telemetry.addLine();

        // Initialize Helpers
        try {
            dashboard = FtcDashboard.getInstance();
            dashboard.startCameraStream(streamProc, 0);
            dashboard.clearTelemetry();

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

            boolean cameraStreaming = false;
            long startCameraWait = System.currentTimeMillis();
            boolean timedOut = false;

            while (!cameraStreaming && !timedOut)  {
                cameraStreaming = (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING);
                timedOut = (System.currentTimeMillis() - (startCameraWait + PARAMS.cameraStreamingWait)) > 0;
                SystemClock.sleep(20);
            }

            if (!cameraStreaming)
                success = false;
            else {
                if (PARAMS.cameraManualExposure) {
                    ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
                    if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                        exposureControl.setMode(ExposureControl.Mode.Manual);
                    }

                    GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
                    gainControl.setGain(PARAMS.cameraExposureGain);
                }

                visionPortal.setProcessorEnabled(tfod, true);
            }
        }  catch (Exception e) {
            success = false;
        }

        if (success) {
            telemetry.addLine("All Sensors Initialized");
            telemetry.addLine("");
            telemetry.addData(">", "Press Play to Start");
        } else {
            telemetry.addLine("");
            telemetry.addLine("*** INITIALIZATION FAILED ***");
        }
        telemetry.update();
        return (success);
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
