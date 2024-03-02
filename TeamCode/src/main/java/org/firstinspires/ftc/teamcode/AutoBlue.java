package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
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
        public String versionNum = "4.1.20";
        public boolean frontStage = true;
        public boolean ifSafe = false;
        public int tfodWaitMS = 3000;
        public int PartnerWaitTime = 500;
        public int sensorRangeTime = 500;
        public double sensorRangeValue = 2;
        public double sensorGainValueForward = 0.1;
        public double sensorGainValueRotation = 0.03;
    }

    public static Params PARAMS = new Params();
    private FtcDashboard dashboard;
    private MecanumDrive drive;
    private ClawMoves whiteClaw;
    private Conveyor whiteConveyor;
    private DistanceSystem distSys;
    private TensorFlow tenFl;
    private int propSpikeMark = 0;

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

        waitForStart();
        telemetry.clear();
        if (isStopRequested()) return;

        // Detect Object with Tensor Flow
        propSpikeMark = tfodSelectSpikeMark();
        updateTelemetry();

        toSpikeMark(propSpikeMark);
        if (PARAMS.frontStage) {
            toPixelStack();
            toFrontPanel(propSpikeMark);
            AutoCommon.PlacePixel(false, drive, whiteClaw, whiteConveyor);
        } else {
            toBackPanel(propSpikeMark);
            AutoCommon.PlacePixel(true, drive, whiteClaw, whiteConveyor);
        }

        if (!PARAMS.frontStage) {
            if (PARAMS.ifSafe) {
                toSafety();
            } else {
                secondHalfBack(propSpikeMark);
            }
        }
    }

    private int tfodSelectSpikeMark() {
        long endDetectMS  = System.currentTimeMillis() + PARAMS.tfodWaitMS;
        boolean timeExpired = false;
        int spikeMark = 3;

        while (opModeIsActive() && (spikeMark == 3)  && (!timeExpired)) {
            spikeMark = tenFl.DetectProp();

            timeExpired = (System.currentTimeMillis() > endDetectMS);
            if ((spikeMark == 3) && !timeExpired)
                sleep( 50);
        }
        tenFl.CleanUp();

        return spikeMark;
    }

    //to the spike mark
    private void toSpikeMark(int spike) {
        double X, Y, ang;

        if (PARAMS.frontStage) {
            // FRONT STAGE  - Go to specified spikeMark and Line Up White Pixel Stack
            if (spike == 1) {
                X = 25; Y = -6; ang = 0;
            } else if(spike == 2) {
                X = 22.0; Y = -5.3; ang = 0;
            } else {
                X = 14; Y = -3.0; ang = -28.0;
            }


            if (spike == 1) {
                // Spike 1 - Avoid Gates on Left
                Action moveOneSMPlan = drive.actionBuilder(drive.pose)
                        .splineTo(new Vector2d(X, Y),Math.toRadians(ang))
                        .turn(Math.toRadians(90))
                        .build();

                Action moveBack = drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .lineToX(17)
                        .turnTo(Math.toRadians(-90))
                        .build();

                Actions.runBlocking(new SequentialAction(moveOneSMPlan,
                        whiteClaw.PlacePixelAction(),
                        new ParallelAction(moveBack, whiteClaw.RetractArmAction())));
            } else if (spike == 2) {
                Action moveDropPixel = drive.actionBuilder(drive.pose)
                        .splineTo(new Vector2d(X, Y), Math.toRadians(ang))
                        .build();

                Action moveBack = drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .lineToX(20)
                        .turnTo(Math.toRadians(-90))
                        .build();

                Actions.runBlocking(new SequentialAction(moveDropPixel,
                        whiteClaw.PlacePixelAction(),
                        new ParallelAction(moveBack, whiteClaw.RetractArmAction())));
            } else {
                //  Pixel 3
                Action moveDropPixel = drive.actionBuilder(drive.pose)
                        .splineTo(new Vector2d(X, Y), Math.toRadians(ang))
                        .build();

                Action moveBack = drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .splineTo(new Vector2d(14, 0), Math.toRadians(0))
                        .turnTo(Math.toRadians(-90))
                        .build();

                Actions.runBlocking(new SequentialAction(moveDropPixel,
                        whiteClaw.PlacePixelAction(),
                        new ParallelAction(moveBack, whiteClaw.RetractArmAction())));
            }
        } else {
            // BACK STAGE - Go to specified spikeMark and Line Up to Backdrop
            if (spike == 1) {
                X = 13.5; Y = 3.0; ang = 32.0;
            } else if(spike == 2) {
                X = 23.0; Y = 6; ang = 0;
            } else {
                X = 28; Y = 4; ang = 0;
            }

            if (spike == 1 || spike == 2) {
                Action moveRb = drive.actionBuilder(drive.pose)
                        .splineTo(new Vector2d(X, Y), Math.toRadians(ang))
                        .build();
                Actions.runBlocking(new SequentialAction(moveRb, whiteClaw.PlacePixelAction()));

                //steps back from the spike mark
                Action moveBack = drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .splineTo(new Vector2d(11, 6), Math.toRadians(90))
                        .build();
                Actions.runBlocking(new ParallelAction(moveBack, whiteClaw.RetractArmAction()));
            } else {
                // Spike 3 - Avoid Gates on Right
                Action moveThirdSMPlan = drive.actionBuilder(drive.pose)
                        .splineTo(new Vector2d(X, Y),Math.toRadians(ang))
                        .turn(Math.toRadians(-90))
                        .build();
                Actions.runBlocking(new SequentialAction(moveThirdSMPlan, whiteClaw.PlacePixelAction()));
            }
        }
        drive.updatePoseEstimate();
    }

    //goes front to the pixel (when it started from the frontStage)
    private void toPixelStack() {
        Action moveToStack = drive.actionBuilder(drive.pose)
                .setReversed(false)
                .splineTo(new Vector2d(28, -18), Math.toRadians(-91))
                .strafeTo(new Vector2d(39.0, -18))
                .build();
        Actions.runBlocking(new SequentialAction( new ParallelAction(moveToStack, whiteClaw.RetractArmAction()),
                whiteClaw.TopOfStackPickupAction(4) ));

        whiteClaw.closeGrip();
        drive.updatePoseEstimate();
    }


    // Move to the Backdrop from Frontstage
    private void toFrontPanel( int spikeMark) {
        double targetX = 36;
        if (spikeMark == 3)
            targetX = 38;
        else if (spikeMark == 1)
            targetX = 25.0;
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
                .splineTo(new Vector2d(targetX, 87), Math.toRadians(90))
                .build();
        Actions.runBlocking(backdrop);
        drive.updatePoseEstimate();
    }


    //to the panel in the back
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
                .strafeTo(new Vector2d(6, 36.0))
                .build();
        Actions.runBlocking(secMoveToSafety);
    }

    //goes front to the pixels (when it started from backStage)
    private void secondHalfBack(int spikeMark){
        Action moveToPixels = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(50, -10), Math.toRadians(-90))
                .splineTo(new Vector2d(48, -50), Math.toRadians(-91))
                .build();
        Actions.runBlocking(new ParallelAction(moveToPixels, whiteClaw.RetractArmAction()));

        // TODO:  Add Code to Navigate to Pixel Stack and Pickup
        /*
        Action moveToStack = drive.actionBuilder(drive.pose)
                .setReversed(false)
                .splineTo(new Vector2d(28, -18), Math.toRadians(-91))
                .strafeTo(new Vector2d(41, -18.75))
                .build();
        Actions.runBlocking(new SequentialAction( new ParallelAction(moveToStack, whiteClaw.RetractArmAction()),
                whiteClaw.TopOfStackPickupAction(4) ));

        whiteClaw.closeGrip();
        drive.updatePoseEstimate();
         */

        // TODO:  Add Code to Navigate Back to Board
        /*
        double targetX = 36;
        if (spikeMark == 3)
            targetX = 38;
        else if (spikeMark == 1)
            targetX = 25.0;
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
                .splineTo(new Vector2d(targetX, 87), Math.toRadians(90))
                .build();
        Actions.runBlocking(backdrop);
        drive.updatePoseEstimate();
         */
    }





    //going through the middle gate (frontStage)
    private void updateTelemetry() {
        telemetry.addLine("TensorFlow");
        telemetry.addLine().addData("Prop Mark", propSpikeMark );
        telemetry.addLine().addData("Objects", tenFl.tlmObjectCnt);
        telemetry.addLine().addData("Confidence", tenFl.tlmConfidence);
        telemetry.addLine().addData("Obj X", tenFl.tlmBestPropXPos);
        telemetry.addLine().addData("Obj Y", tenFl.tlmBestPropYPos);
        telemetry.update();

        // FTC Dashboard Telemetry
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Prop Mark",  propSpikeMark);
        packet.put("Objects", tenFl.tlmObjectCnt);
        packet.put("Confidence", tenFl.tlmConfidence);
        packet.put("Obj X", tenFl.tlmBestPropXPos);
        packet.put("Obj Y", tenFl.tlmBestPropYPos);
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
