package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helper.AutoCommon;
import org.firstinspires.ftc.teamcode.Helper.ClawMoves;
import org.firstinspires.ftc.teamcode.Helper.Conveyor;
import org.firstinspires.ftc.teamcode.Helper.DistanceSystem;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.helper.TargetPose;
import org.firstinspires.ftc.teamcode.Helper.TensorFlow;
@Config
@Autonomous (name = "Auto Blue", group = "RoadRunner")
public class AutoBlue extends LinearOpMode {
    /*
     *  FTC Dashboard Parameters
     */
    public static class Params {
        public String versionNum = "4.1.31";
        public boolean frontStage = true;
        public boolean ifSafe = true;
        public int PartnerWaitTime = 0;
        public int sensorRangeTime = 500;
        public double sensorRangeValue = 2;
        public double sensorGainValueForward = 0.1;
        public double sensorGainValueRotation = 0.03;
        public double toFrontPixelStackX = 51.75;
        public double toFrontPixelStackY = -18.75;
        public double toPixYBack = -78.0;
    }

    public static Params PARAMS = new Params();
    private FtcDashboard dashboard;
    private MecanumDrive drive;
    private ClawMoves whiteClaw;
    private Conveyor whiteConveyor;
    private DistanceSystem distSys;
    private TensorFlow tenFl;
    private int propSpikeMark = 3;

    @Override
    public void runOpMode() {
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
            distSys = new DistanceSystem(hardwareMap);
            whiteClaw.AutonomousStart();
            tenFl = new TensorFlow(hardwareMap);

            boolean cameraStreaming = false;
            long startCameraWait = System.currentTimeMillis();
            boolean timedOut = false;

            while (!cameraStreaming && !timedOut)  {
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
            telemetry.addData("Exeption", e.toString());
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

        toSpikeMark(propSpikeMark);
        if (PARAMS.frontStage) {
            toPixelStackFront();
            toFrontPanel(propSpikeMark);
            AutoCommon.PlacePixel(true, false, drive, whiteClaw, whiteConveyor);
        } else {
            toBackPanel(propSpikeMark);
            AutoCommon.PlacePixel(true, true, drive, whiteClaw, whiteConveyor);
        }
      if (!PARAMS.frontStage) {
            if (PARAMS.ifSafe) {
                toSafety();
            } else {
                secondHalfBackStage(propSpikeMark);
            }
        }
    }
    //to the spike mark
    private void toSpikeMark(int spike) {
        if (PARAMS.frontStage) {
            Action moveToSpike;
            if (spike == 1) {
                moveToSpike = drive.actionBuilder(drive.pose)
                        .splineTo(new Vector2d(25.5, -6.5), Math.toRadians(0))
                        .turnTo(Math.toRadians(90))
                        .build();
            } else if(spike == 2) {
                moveToSpike = drive.actionBuilder(drive.pose)
                        .splineTo(new Vector2d(23, -5.3), Math.toRadians(0))
                        .build();
            } else {
                moveToSpike = drive.actionBuilder(drive.pose)
                        .splineTo(new Vector2d(14, -3), Math.toRadians(-28))
                        .build();
            }
            Actions.runBlocking(new SequentialAction(moveToSpike, whiteClaw.PlacePixelAction()));
            drive.updatePoseEstimate();

            Action moveAway;
            if (spike == 1) {
                moveAway = drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .lineToX(17)
                        .turnTo(Math.toRadians(-90))
                        .build();
            } else if(spike == 2) {
                moveAway = drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .lineToX(20)
                        .turnTo(Math.toRadians(-90))
                        .setReversed(false)
                        .splineTo(new Vector2d(28, -18),Math.toRadians(-91))
                        .strafeTo(new Vector2d(39.75, PARAMS.toFrontPixelStackY))
                        .build();
            } else {
                moveAway = drive.actionBuilder(drive.pose)
                        .turnTo(Math.toRadians(0))
                        .waitSeconds(1)
                        .splineTo(new Vector2d(PARAMS.toFrontPixelStackX, 0), Math.toRadians(0))
                        .turnTo(Math.toRadians(-90))
                        .build();
            }
            Actions.runBlocking(new SequentialAction(whiteClaw.RetractArmAction(), moveAway));

        } else {
            // BACK STAGE - Go to specified spikeMark and Line Up to Backdrop
            Action moveToSpike;
            if (spike == 1) {
                moveToSpike = drive.actionBuilder(drive.pose)
                        .splineTo(new Vector2d(12.5, 3), Math.toRadians(32))
                        .build();
            } else if(spike == 2) {
                moveToSpike = drive.actionBuilder(drive.pose)
                        .splineTo(new Vector2d(21.25, 7), Math.toRadians(0))
                        .build();
            } else {
                moveToSpike = drive.actionBuilder(drive.pose)
                        .splineTo(new Vector2d(28, 5.5), Math.toRadians(0))
                        .turnTo(Math.toRadians(-90))
                        .build();

            }
            Actions.runBlocking(new SequentialAction(moveToSpike, whiteClaw.PlacePixelAction()));

            Action moveBack;
            if (spike == 1) {
                moveBack = drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .splineTo(new Vector2d(11, 10), Math.toRadians(90))
                        .build();
            } else if(spike == 2) {
                moveBack = drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .splineTo(new Vector2d(15, 5), Math.toRadians(90))
                        .build();
            } else {
                moveBack = drive.actionBuilder(drive.pose)
                        .turnTo(Math.toRadians(-90))
                        .build();
            }
            Actions.runBlocking(new SequentialAction(whiteClaw.RetractArmAction(), moveBack));
        }
        drive.updatePoseEstimate();
    }

    /*
     *  FRONT Stage Methods
     */

    //goes front to the pixel (when it started from the frontStage)
    private void toPixelStackFront() {
        Action moveCloseToStack = drive.actionBuilder(drive.pose)
                .splineTo( new Vector2d(51.75, PARAMS.toFrontPixelStackY), Math.toRadians(-90), new TranslationalVelConstraint(20))
                .build();
        Actions.runBlocking(new SequentialAction(whiteClaw.PrepForTopOfStackPickupAction(4),
                moveCloseToStack,
                whiteClaw.TopOfStackPickupAction()));

        drive.updatePoseEstimate();
        updateTelemetry();
    }

    // Move to the Backdrop from Frontstage
    private void toFrontPanel( int spikeMark) {
        double targetX = 36;
        if (spikeMark == 3)
            targetX = 42;
        else if (spikeMark == 1)
            targetX = 32.5;
        whiteClaw.RetractArmAction();

        Action moveBar = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(51, -15))
                .setReversed(true)
                .splineTo(new Vector2d(56, 46), Math.toRadians(90))
                .build();
        Actions.runBlocking(new ParallelAction(moveBar, whiteClaw.SuplexPixelAction()));

        sleep(PARAMS.PartnerWaitTime);

        Action backdrop = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineTo(new Vector2d(targetX, 89), Math.toRadians(90))
                .build();
        Actions.runBlocking(backdrop);
        drive.updatePoseEstimate();
    }

    //to the panel from the the backstage
    private void toBackPanel(int spikeMark){
        double targetX = 30;
        if (spikeMark == 3)
            targetX = 38;
        else if (spikeMark == 1)
            targetX = 25.0;

        Action moveRb3 = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineTo(new Vector2d(targetX,40.0), Math.toRadians(89))
                .build();
        Actions.runBlocking(moveRb3);
    }

    private void toSafety() {
        Action secMoveToSafety = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(6, 35.25))
                .build();
        Actions.runBlocking(secMoveToSafety);
    }

    //goes front to the pixels (when it started from backStage)
    private void secondHalfBackStage(int spikeMark) {

        Action moveToPixels = drive.actionBuilder(drive.pose)
           // .setReversed(true)
            .strafeTo(new Vector2d(51.75, 35.25))
            .lineToY(-78)
            .build();
        Actions.runBlocking(moveToPixels);

       Action moveToStack = drive.actionBuilder(drive.pose)
               .splineTo(new Vector2d(51.75, PARAMS.toPixYBack), Math.toRadians(-90))
               .build();
       Actions.runBlocking(new SequentialAction(new ParallelAction(moveToStack, whiteClaw.RetractArmAction()),
               whiteClaw.PrepForTopOfStackPickupAction(4),
               whiteClaw.TopOfStackPickupAction()));

        double targetX = 36;
        if (spikeMark == 3)
            targetX = 38;
        else if (spikeMark == 1)
            targetX = 25.0;
        whiteClaw.RetractArmAction();

        Action moveBar = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .lineToY(35.25)
                .build();
        Actions.runBlocking(new ParallelAction(moveBar, whiteClaw.SuplexPixelAction()));

        sleep(PARAMS.PartnerWaitTime);
        Action backdrop = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineTo(new Vector2d(targetX, 40), Math.toRadians(90))
                .build();
        Actions.runBlocking(backdrop);
        drive.updatePoseEstimate();
        }


    //going through the middle gate (frontStage)
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
        packet.put("Safe Mode", (PARAMS.ifSafe ?"ON" : "OFF"));
        packet.put("Confidence", tenFl.tlmConfidence);
        dashboard.sendTelemetryPacket(packet);
    }

    //Use sensor to square up to the panel
    private void SensorApproach() {
        long timeout = System.currentTimeMillis()+PARAMS.sensorRangeTime;
        TargetPose pose = distSys.getTargetPose(true);  //Get Initial Values

        while (pose.range > (pose.range-PARAMS.sensorRangeValue) && System.currentTimeMillis() < timeout){
            pose = distSys.getTargetPose(false);
            double rangeError = (pose.range-PARAMS.sensorRangeValue);

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            double forward = Range.clip(-rangeError * PARAMS.sensorGainValueForward, -0.3, 0.3);
            double rotate = Range.clip(-pose.yaw * PARAMS.sensorGainValueRotation, -0.25, 0.25);

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(forward, 0), rotate));
            drive.updatePoseEstimate();
            sleep(30);
        }
    }
}
