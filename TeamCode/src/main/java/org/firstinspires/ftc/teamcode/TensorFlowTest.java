package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Helper.ClawMoves;
import org.firstinspires.ftc.teamcode.Helper.TensorFlow;
import org.firstinspires.ftc.teamcode.Helper.gamePadInputV2;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

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
        waitForStart();
        if (isStopRequested() || !good_init)
            return;
        telemetry.clear();

        while (opModeIsActive()) {
            propSpikeMark = tenFl.telemTFOD(1500);
            updateTelemetry();
            sleep(20);
        }
        tenFl.CleanUp();
    }


    private boolean initialize() {
        boolean success = true;

        // Load Introduction and Wait for Start
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.addLine("TensorFlow Helper Test");
        telemetry.addLine();

        // Initialize Helpers
        try {
            tenFl = new TensorFlow(hardwareMap);
            dashboard = FtcDashboard.getInstance();
            dashboard.clearTelemetry();
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
        telemetry.addLine().addData("Prop Mark", propSpikeMark);
        telemetry.addLine().addData("Objects", tenFl.tlmObjectCnt);
        telemetry.addLine().addData("Confidence", tenFl.tlmConfidence);
        telemetry.addLine().addData("Obj X", tenFl.tlmBestObjectX);
        telemetry.addLine().addData("Obj Y", tenFl.tlmBestObjectY);
        telemetry.update();

        // FTC Dashboard Telemetry
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Prop Mark", propSpikeMark);
        packet.put("Objects", tenFl.tlmObjectCnt);
        packet.put("Confidence", tenFl.tlmConfidence);
        packet.put("Obj X", tenFl.tlmBestObjectX);
        packet.put("Obj Y", tenFl.tlmBestObjectY);
        dashboard.sendTelemetryPacket(packet);
    }
}
