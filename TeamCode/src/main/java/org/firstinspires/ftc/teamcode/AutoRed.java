package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helper.AutoCommon;
import org.firstinspires.ftc.teamcode.Helper.ClawMoves;
import org.firstinspires.ftc.teamcode.Helper.Conveyor;
import org.firstinspires.ftc.teamcode.Helper.TensorFlow;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Config
@Autonomous (name = "Auto Red", group = "RoadRunner")
public class AutoRed extends LinearOpMode {
    /*3
     *  FTC Dashboard Parameters
     */
    public static class Params {
        public String versionNum = "4.1.22";
        public boolean frontStage = false;
        public boolean ifSafe = true;
        public int PartnerWaitTime = 0;
    }

    public static Params PARAMS = new Params();
    private FtcDashboard dashboard;
    private MecanumDrive drive;
    private ClawMoves whiteClaw;
    private Conveyor whiteConveyor;
    private TensorFlow tenFl;
    public int propSpikeMark = 3;




    @Override
    public void runOpMode() {
        // Load Introduction and Wait for Start

        // Load Introduction and Wait for Start
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.addLine("RoadRunner Auto Drive BLUE");
        telemetry.addLine();
        telemetry.addLine().addData("Version", PARAMS.versionNum);
        telemetry.addLine();
        telemetry.update();
        // Initialize Helpers
        boolean initialized;
        try {
            dashboard = FtcDashboard.getInstance();
            dashboard.clearTelemetry();

            drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            whiteClaw = new ClawMoves(hardwareMap);
            whiteConveyor = new Conveyor(hardwareMap);
            whiteClaw.AutonomousStart();
            tenFl = new TensorFlow(hardwareMap);

            boolean cameraStreaming = false;
            long startCameraWait = System.currentTimeMillis();
            boolean timedOut = false;

            while (!cameraStreaming && !timedOut) {
                cameraStreaming = tenFl.isCameraStreaming();
                timedOut = (System.currentTimeMillis() - (startCameraWait + 1500)) > 0;
                sleep(20);
            }
            initialized = cameraStreaming;
            if (initialized)
                telemetry.addData(">", "Press Start to Launch");
            else
                telemetry.addLine("*** TFOD CAMERA OFFLINE ***");
        } catch (Exception e) {
            initialized = false;
            telemetry.addLine("");
            telemetry.addLine("*** INITIALIZATION FAILED ***");
            telemetry.addData("Exception", e.toString());
        }

        telemetry.update();
        if (!initialized) return;
        while (!isStopRequested() && !opModeIsActive()) {
            // Detect Object with Tensor Flow
            propSpikeMark = tenFl.DetectProp();
            updateTelemetry();
            if (!isStopRequested() && !opModeIsActive())
                sleep(100);  // Free up Processor
        }

        waitForStart();
        telemetry.clear();

        if (isStopRequested()) return;

        updateTelemetry();
        tenFl.CleanUp();

        toSpikeMark(propSpikeMark);

        if (!PARAMS.frontStage) {
            toBackPanel(propSpikeMark);
            AutoCommon.PlacePixel(false, true, drive, whiteClaw, whiteConveyor);
            drive.updatePoseEstimate();
            updateTelemetry();
            if (PARAMS.ifSafe)
                toSafety();
            else {
                toPixelStack();
                BackToBackdrop();
                sleep(1000);
            }
        }else{
            toPixelStackFront();
            toFrontPanel(propSpikeMark);
            AutoCommon.PlacePixel(false, false, drive, whiteClaw, whiteConveyor);
        }
    }


    private void toSpikeMark(int spike) {
        if (PARAMS.frontStage) {
            // FRONT STAGE  - Go to specified spikeMark and Line Up White Pixel Stack
            Action moveToSpike;
            if (spike == 3) {
                moveToSpike = drive.actionBuilder(drive.pose)
                        .splineTo(new Vector2d(27, 5.5), Math.toRadians(0))
                        .turnTo(Math.toRadians(-90))
                        .build();
            } else if(spike == 2) {
                moveToSpike = drive.actionBuilder(drive.pose)
                        .splineTo(new Vector2d(23, 5.5), Math.toRadians(0))
                        .build();
            } else {
                moveToSpike = drive.actionBuilder(drive.pose)
                        .splineTo(new Vector2d(13.5, 4), Math.toRadians(30))
                        .build();
            }
            Actions.runBlocking(new SequentialAction(moveToSpike, whiteClaw.PlacePixelAction()));

            Action moveAway;
            if (spike == 3) {
                moveAway = drive.actionBuilder(drive.pose)
                        .turnTo(Math.toRadians(0))
                        .splineTo(new Vector2d(49.5,0), Math.toRadians(90))
                        .build();
            } else if(spike == 2) {
                moveAway = drive.actionBuilder(drive.pose)
                        .turnTo(Math.toRadians(90))
                        .splineTo(new Vector2d(28, 18), Math.toRadians(0))
                        .splineTo(new Vector2d(33, 18), Math.toRadians(0))
                        .splineTo(new Vector2d(49.5, 14.5), Math.toRadians(90))
                        .build();
            } else {
                moveAway = drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .splineTo(new Vector2d(10, 0), Math.toRadians(180))
                        .setReversed(false)
                        .lineToX(49.5)
                        .turnTo(Math.toRadians(90))
                        .build();
            }
            Actions.runBlocking(new SequentialAction(whiteClaw.RetractArmAction(), moveAway));
        } else {
            // BACK STAGE - Go to specified spikeMark and Line Up to Backdrop
            Action moveToSpike;
            if (spike == 1) {
                moveToSpike = drive.actionBuilder(drive.pose)
                        .splineTo(new Vector2d(26, -5.5), Math.toRadians(0))
                        .turnTo(Math.toRadians(90))
                        .build();
            } else if(spike == 2) {
                moveToSpike = drive.actionBuilder(drive.pose)
                        .splineTo(new Vector2d(23, -4), Math.toRadians(0))
                        .build();
            } else {
                moveToSpike = drive.actionBuilder(drive.pose)
                        .splineTo(new Vector2d(14.5, -4), Math.toRadians(-25))
                        .build();

            }
            Actions.runBlocking(new SequentialAction(moveToSpike, whiteClaw.PlacePixelAction()));


            Action moveBack;
            if (spike == 1) {
                moveBack = drive.actionBuilder(drive.pose)
                        .turnTo(Math.toRadians(90))
                        .build();
            } else if(spike == 2) {
                moveBack = drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .splineTo(new Vector2d(15, -5), Math.toRadians(-90))
                        .build();
            } else {
                moveBack = drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .splineTo(new Vector2d(10, -10), Math.toRadians(-90))
                        .build();
            }
            Actions.runBlocking(new SequentialAction(whiteClaw.RetractArmAction(), moveBack));
        }
        drive.updatePoseEstimate();
    }

    /*
     *  FRONT Stage Methods
     */

    public void toPixelStackFront () {
        if (PARAMS.ifSafe)
            return;
        Action moveCloseToStack = drive.actionBuilder(drive.pose)
                .splineTo( new Vector2d(49.5, 17.5), Math.toRadians(90), new TranslationalVelConstraint(20))
                .build();
        Actions.runBlocking(new SequentialAction(whiteClaw.PrepForTopOfStackPickupAction(3),
                moveCloseToStack,
                whiteClaw.TopOfStackPickupAction()));

        drive.updatePoseEstimate();
        updateTelemetry();
    }

    // Move to the Backdrop from Frontstage
    private void toFrontPanel(int spikeMark) {
        double[] dropPosX = {0.0, 34, 27.5, 22};
        double posX = dropPosX[spikeMark];

        whiteClaw.RetractArmAction();

        Action moveBar = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(51, 15))
                .setReversed(true)
                .splineTo(new Vector2d(54, -46), Math.toRadians(-90))
                .build();
        Actions.runBlocking(new ParallelAction(moveBar, whiteClaw.SuplexPixelAction()));

        sleep(PARAMS.PartnerWaitTime);

        Action backdrop = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineTo(new Vector2d(posX, -87.5), Math.toRadians(-90))
                .build();
        Actions.runBlocking(backdrop);
        drive.updatePoseEstimate();
    }


    /*
     *  BACK Stage Methods
     */
    public void toPixelStack() {
        // cross field and prep claw
        Action moveAcrossField = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(51, -5),Math.toRadians(90))
                .splineTo(new Vector2d(47.5, 61.75),Math.toRadians(90))
                .build();
        Actions.runBlocking(new SequentialAction(whiteClaw.RetractArmAction(), moveAcrossField,
                whiteClaw.PrepForTopOfStackPickupAction(3)));
        drive.updatePoseEstimate();

        // slow approach pixel stack
        Action moveCloseToStack = drive.actionBuilder(drive.pose)
                .splineTo( new Vector2d(47.5, 64.5), Math.toRadians(90), new TranslationalVelConstraint(20))
                .build();
        Actions.runBlocking(new SequentialAction(moveCloseToStack, whiteClaw.TopOfStackPickupAction()));

        drive.updatePoseEstimate();
        updateTelemetry();
    }

    private void  BackToBackdrop () {
        //drops white pixels in the backdrop
        double backDropPosition = ((propSpikeMark==1) ? 22:34);

        Action moveToBackdrop = drive.actionBuilder(drive.pose)
               .setReversed(true)
               .splineTo(new Vector2d(51, -6), Math.toRadians(-90),null)
               .splineTo(new Vector2d(backDropPosition, -40.5), Math.toRadians(-90),new TranslationalVelConstraint(20))
               .build();
        Actions.runBlocking(new ParallelAction(whiteClaw.SuplexPixelAction(),moveToBackdrop ));

        drive.updatePoseEstimate();
        updateTelemetry();
        AutoCommon.PlacePixel(false, false, drive, whiteClaw, whiteConveyor);
    }

    //to the panel in the back
    private void toBackPanel(int spikeMark) {
        double[] dropPosX = {0.0, 34, 27.5, 22};
        Action moveRb3 = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineTo(new Vector2d(dropPosX[spikeMark],-41.0), Math.toRadians(-89))
                .build();
        Actions.runBlocking(moveRb3);
    }

    private void toSafety() {
        Action secMoveToSafety = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(6, -36.5))
                .build();
        Actions.runBlocking(secMoveToSafety);
    }


    private void updateTelemetry() {
        telemetry.addLine("RoadRunner Auto Drive BLUE");
        telemetry.addLine();
        telemetry.addLine().addData("Version", PARAMS.versionNum);
        telemetry.addLine();
        telemetry.addLine().addData("Position", (PARAMS.frontStage ? "FRONT Stage" : "BACK Stage"));
        telemetry.addLine().addData("Prop Mark", propSpikeMark );
        telemetry.addLine().addData("Safe Mode", (PARAMS.ifSafe ?"ON" : "OFF"));
        telemetry.addLine().addData("Partner Wait", PARAMS.PartnerWaitTime);
        telemetry.update();

        // FTC Dashboard Telemetry
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Prop Mark",  propSpikeMark);
        packet.put("Position", (PARAMS.frontStage ? "FRONT Stage" : "BACK Stage"));
        packet.put("Confidence", tenFl.tlmConfidence);
        packet.put("Safe Mode", (PARAMS.ifSafe ?"ON" : "OFF"));
        dashboard.sendTelemetryPacket(packet);
    }
}
