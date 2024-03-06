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
       // public double propSpikeMark = 2;    //  Which Spike Mark is the Prop Located on
        public boolean partnerDead = true;
        public boolean frontStage = false;
        public boolean ifSafe = true;
        public int dTime = 500;
        public double rangeNum = 2.0;
        public int rangeTime = 500;
        public double gainValueForward = 0.1;
        public double rangeValue = 2;
        public double gainValueRotation = 0.03;
        public String versionNum = "4.1.2";
        public double toPixY = 18.75;
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

        if (!PARAMS.frontStage) {
            toSpikeMark(propSpikeMark);
            toBackPanel(propSpikeMark);
            AutoCommon.PlacePixel(false, true, drive, whiteClaw, whiteConveyor);
            drive.updatePoseEstimate();
            updateTelemetry();
            if (PARAMS.ifSafe)
                toSafety();
            else {
                toPixelStack();
                sleep(1000);
            }

        }
    }




        /*
        switch(propSpikeMark){
            case 3:
                //toSpikeMark(20.5,4.0,27, PARAMS.frontStage);
                toSpikeMark(14.5,4.5,27, PARAMS.frontStage);
                if(PARAMS.frontStage){
                    toFrontPanel(36.5, PARAMS.partnerDead);
                }
                else{
                    toBackPanel(36.5);
                }
                break;
            case 1:
                //toSpikeMark(18.5, -3.0, -30, PARAMS.frontStage);
                toSpikeMark(16.5, -0.5, -30, PARAMS.frontStage);
               // firstsp();
                if(PARAMS.frontStage){
                    toFrontPanel(25.0, PARAMS.partnerDead);
                }
                else{
                    toBackPanel(25.0);
                }
                break;
            default:
                toSpikeMark(22.0, -3.2,0, PARAMS.frontStage);
                if(PARAMS.frontStage){
                    toFrontPanel(29.0, PARAMS.partnerDead);
                }
                else{
                    toBackPanel(29.0);
                }
                break;
        }

         */


    public void firstsp(){
        Action movethirdSMPlan = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(19.5, 0),Math.toRadians(0)  )
                .splineTo(new Vector2d(19.5, -4.5), Math.toRadians(90))
                .build();
        Actions.runBlocking(new SequentialAction(movethirdSMPlan, whiteClaw.PlacePixelAction()));

        Action moveBack = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineTo(new Vector2d(6, 0), Math.toRadians(-30))
                .build();
        Actions.runBlocking(new ParallelAction(moveBack, whiteClaw.RetractArmAction()));
    }

    public void toPixelStack() {
        // cross field and prep claw
        Action moveAcrossField = drive.actionBuilder(drive.pose)
                    .splineTo(new Vector2d(51, -5),Math.toRadians(90))
                    .splineTo(new Vector2d(47.5, 62),Math.toRadians(90))
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

    //to the spike mark
    /*
    public void toSpikeMark(double X, double Y, int ang, boolean position){
        double an;

        if(position){
            an = 180;
        }
        else{
            an = -90;
        }

        Action moveRb = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(X, Y), Math.toRadians(ang))
                .build();
        Actions.runBlocking(new SequentialAction(moveRb, whiteClaw.PlacePixelAction()));

        if(!PARAMS.frontStage){
            sleep(850);
            whiteClaw.RetractArmAction();
        }

        Action moveBack = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineTo(new Vector2d(6, 0), Math.toRadians(an))
                .build();
        Actions.runBlocking(new ParallelAction(moveBack, whiteClaw.RetractArmAction()));
    }
    */

    //to the panel in the front
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
            sleep(PARAMS.dTime);

            Action backdrop = drive.actionBuilder(drive.pose)
                    .setReversed(true)
                    .splineTo(new Vector2d(targetX, -86), Math.toRadians(-90))
                    .build();
            Actions.runBlocking(backdrop);
        }
    }




    private void toSpikeMark(int spike) {
        double X, Y, ang;

        if (PARAMS.frontStage) {
            // FRONT STAGE  - Go to specified spikeMark and Line Up White Pixel Stack
            if (spike == 1) {
                X = 25.5; Y = 6.5; ang = 0;
            } else if(spike == 2) {
                X = 23.0; Y = 5.3; ang = 0;
            } else {
                X = 14; Y = 3.0; ang = 28.0;
            }

            if (spike == 1) {
                // Spike 1 - Avoid Gates on Left
                Action moveOneSMPlan = drive.actionBuilder(drive.pose)
                        .splineTo(new Vector2d(X, Y),Math.toRadians(ang))
                        .turn(Math.toRadians(-90))
                        .build();

                Action moveBack = drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .lineToX(17)
                        .turnTo(Math.toRadians(90))
                        .build();
                Actions.runBlocking(new SequentialAction(moveOneSMPlan,
                        whiteClaw.PlacePixelAction(),
                        new ParallelAction(moveBack, whiteClaw.RetractArmAction())));
            } else{
                Action moveDropPixel = drive.actionBuilder(drive.pose)
                        .splineTo(new Vector2d(X, Y), Math.toRadians(ang))
                        .build();
                Actions.runBlocking(new SequentialAction(moveDropPixel, whiteClaw.PlacePixelAction(), whiteClaw.RetractArmAction()));


                if(spike == 2){
                    Action moveBackTwo = drive.actionBuilder(drive.pose)
                            .setReversed(true)
                            .lineToX(20)
                            .turnTo(Math.toRadians(90))
                            .build();
                    Actions.runBlocking(new SequentialAction(moveBackTwo, whiteClaw.RetractArmAction()));}

                else if(spike == 3){
                    Action moveBackThree = drive.actionBuilder(drive.pose)
                            .setReversed(true)
                            //.splineTo(new Vector2d(14, 0), Math.toRadians(0))
                            .turnTo(Math.toRadians(90))
                            .lineToY(18.75)
                            .build();
                    Actions.runBlocking(new SequentialAction(moveBackThree, whiteClaw.RetractArmAction()));}

               /* Actions.runBlocking(new SequentialAction(moveDropPixel,
                        whiteClaw.PlacePixelAction()));
                        //new ParallelAction( whiteClaw.RetractArmAction())));*/
            }
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


    //to the panel in the back
    private void toBackPanel(int spikeMark) {
        double[] dropPosX = {0.0, 32, 25, 22};
        Action moveRb3 = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineTo(new Vector2d(dropPosX[spikeMark],-40.0), Math.toRadians(-89))
                .build();
        Actions.runBlocking(moveRb3);
    }


    private void toSafety() {
        Action secMoveToSafety = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(6, -35.25))
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
