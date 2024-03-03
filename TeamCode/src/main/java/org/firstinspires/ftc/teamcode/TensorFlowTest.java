package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helper.TensorFlow;
import org.firstinspires.ftc.vision.VisionPortal;

/*
 * This code was derived from FIRST External Sample ConceptTensorFlowObjectDetection.java
 *
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name= "TensorFlow Helper Test", group="Hardware")
public class TensorFlowTest extends LinearOpMode {
    private TensorFlow tenFl;
    private FtcDashboard dashboard;
    private int propSpikeMark = 0;
    @Override
    public void runOpMode() {
        boolean good_init = initialize();

        while (good_init && !isStopRequested() && !opModeIsActive()) {
            propSpikeMark = tenFl.DetectProp();
            updateTelemetry();
            sleep(100);  // Share the Processor
        }

        waitForStart();
        if (isStopRequested() || !good_init)
            return;
        telemetry.clear();

        while (opModeIsActive()) {
            propSpikeMark = tenFl.DetectProp();
            updateTelemetry();
            sleep(200); // Share the Processor
        }
        tenFl.CleanUp();
    }


    private boolean initialize() {
        boolean success;

        // Load Introduction and Wait for Start
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.addLine("TensorFlow Helper Test");
        telemetry.addLine();

        // Initialize Helpers
        try {
            dashboard = FtcDashboard.getInstance();
            dashboard.clearTelemetry();
            tenFl = new TensorFlow(hardwareMap);

            boolean cameraStreaming = false;
            long startCameraWait = System.currentTimeMillis();
            boolean timedOut = false;

            while (!cameraStreaming && !timedOut)  {
                cameraStreaming = tenFl.isCameraStreaming();
                timedOut = (System.currentTimeMillis() - (startCameraWait + 1500)) > 0;
                SystemClock.sleep(20);
            }
            success = cameraStreaming;
        } catch (Exception e) {
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


    private void updateTelemetry() {
        telemetry.addLine("TensorFlow");
        telemetry.addLine().addData("Spike Mark", propSpikeMark);
        telemetry.addData("Objects/Props", "%01d / %01d", tenFl.tlmObjectCnt, tenFl.tlmPropCnt);
        telemetry.addLine().addData("Confidence", tenFl.tlmConfidence);
        telemetry.addData("Best Prop Pos", "%.0f / %.0f", tenFl.tlmBestPropXPos, tenFl.tlmBestPropYPos);
        telemetry.update();

        // FTC Dashboard Telemetry
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Prop Mark", propSpikeMark);
        packet.put("Objects", tenFl.tlmObjectCnt);
        packet.put("Confidence", tenFl.tlmConfidence);
        packet.put("Obj X", tenFl.tlmBestPropXPos);
        packet.put("Obj Y", tenFl.tlmBestPropYPos);
        dashboard.sendTelemetryPacket(packet);
    }
}
