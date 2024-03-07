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
import org.firstinspires.ftc.teamcode.Helper.DistanceSystem;
import org.firstinspires.ftc.teamcode.Helper.DrivetrainV2;
import org.firstinspires.ftc.teamcode.Helper.TensorFlow;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Config
@Autonomous (name = "Auto Red", group = "RoadRunner")
public class AutoRed extends LinearOpMode {
    /*
     *  FTC Dashboard Parameters
     */
    public static class Params {

        public String versionNum = "4.1.7";
        public boolean frontStage = false;
        public boolean ifSafe = true;
        public int PartnerWaitTime = 500;
    }

    public static Params PARAMS = new Params();
    private FtcDashboard dashboard;
    private MecanumDrive drive;
    private ClawMoves whiteClaw;
    private Conveyor whiteConveyor;
    private DistanceSystem distSys;
    private DrivetrainV2 drv;
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
            distSys = new DistanceSystem(hardwareMap);
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
            telemetry.addData("Exeption", e.toString());
        }

        telemetry.update();
        if (!initialized) return;
        while (!isStopRequested() && !opModeIsActive() && propSpikeMark == 3) {
            // Detect Object with Tensor Flow
            propSpikeMark = tenFl.DetectProp();
            updateTelemetry();
            if (propSpikeMark == 3)
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


        }
    }

    public void toPixelStackFront () {
        Action moveCloseToStack = drive.actionBuilder(drive.pose)
                .splineTo( new Vector2d(50, 21), Math.toRadians(90), new TranslationalVelConstraint(20))
                .build();
        Actions.runBlocking(new SequentialAction(whiteClaw.PrepForTopOfStackPickupAction(4),moveCloseToStack, whiteClaw.TopOfStackPickupAction()));

        drive.updatePoseEstimate();
        updateTelemetry();

    }

    private void toSpikeMark(int spike) {
        double X, Y, ang;

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
                        .splineTo(new Vector2d(23, 5), Math.toRadians(0))
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
                        .splineTo(new Vector2d(50,0), Math.toRadians(90))
                        .build();
            } else if(spike == 2) {
                moveAway = drive.actionBuilder(drive.pose)
                        .turnTo(Math.toRadians(90))
                        .splineTo(new Vector2d(28, 18), Math.toRadians(90))
                        .strafeTo(new Vector2d(50, 18))
                        .build();
            } else {
                moveAway = drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .splineTo(new Vector2d(10, 0), Math.toRadians(180))
                        .setReversed(false)
                        .lineToX(50)
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
        AutoCommon.PlacePixel(false, true, drive, whiteClaw, whiteConveyor);
     }

    //to the panel in the back
    private void toBackPanel(int spikeMark) {
        double[] dropPosX = {0.0, 34, 28, 22};
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




    // Move to the Backdrop from Frontstage
    private void toFrontPanel( int spikeMark) {
        double targetX = 36;
        if (spikeMark == 3)
            targetX = 42;
        else if (spikeMark == 1)
            targetX = 32.5;
        whiteClaw.RetractArmAction();

        Action moveBar = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(51, 15))
                .setReversed(true)
                .splineTo(new Vector2d(56, -46), Math.toRadians(-90))
                .build();
        Actions.runBlocking(new ParallelAction(moveBar, whiteClaw.SuplexPixelAction()));

        sleep(PARAMS.PartnerWaitTime);

        Action backdrop = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineTo(new Vector2d(targetX, -87), Math.toRadians(-90))
                .build();
        Actions.runBlocking(backdrop);
        drive.updatePoseEstimate();
    }

    public void toFrontPanel( double targetX, boolean partDead) {

        whiteClaw.RetractArmAction();

        Action moveBar = drive.actionBuilder(drive.pose)
                .turnTo(Math.toRadians(90))
                .lineToY(-40)
                .build();
        Actions.runBlocking(moveBar);

        if(partDead){
            Action backdrop = drive.actionBuilder(drive.pose)
                    .setReversed(true)
                    .splineTo(new Vector2d(targetX, -60), Math.toRadians(-90))
                    .splineTo(new Vector2d(targetX, -86), Math.toRadians(-90))
                    .build();
            Actions.runBlocking(backdrop);
        }

        else {
            sleep(PARAMS.PartnerWaitTime);

            Action backdrop = drive.actionBuilder(drive.pose)
                    .setReversed(true)
                    .splineTo(new Vector2d(targetX, -86), Math.toRadians(-90))
                    .build();
            Actions.runBlocking(backdrop);
        }
    }



    private void updateTelemetry() {
        telemetry.addLine("RoadRunner Auto Drive BLUE");
        telemetry.addLine();
        telemetry.addLine().addData("Version", PARAMS.versionNum);
        telemetry.addLine();
        telemetry.addLine().addData("Position", (PARAMS.frontStage ? "FRONT Stage" : "BACK Stage"));
        telemetry.addLine().addData("Prop Mark", propSpikeMark );
        telemetry.addLine().addData("Safe Mode", (PARAMS.ifSafe ?"ON" : "OFF"));




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
}
