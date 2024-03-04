package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
        public int dTime = 500;
        public double rangeNum = 2.0;
        public int rangeTime = 500;
        public double gainValueForward = 0.1;
        public double rangeValue = 2;
        public double gainValueRotation = 0.03;
        public String versionNum = "3.4";
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
        while (!isStopRequested() && !opModeIsActive() && propSpikeMark == 3) {
            // Detect Object with Tensor Flow
            propSpikeMark = tenFl.DetectProp();
            updateTelemetry();
            if (propSpikeMark == 3)
                sleep( 100);  // Free up Processor
        }


        waitForStart();
        telemetry.clear();
        if (isStopRequested()) return;

        propSpikeMark = tenFl.DetectProp();
        telemetry.clear();
        updateTelemetry();
        tenFl.CleanUp();

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
        whiteClaw.PrepForPixel(false);

        whiteConveyor.moveViperToPosition(700);
        sleep(1000);
        whiteConveyor.moveConvForward();
        sleep(1500);
        whiteConveyor.stopConv();
        whiteConveyor.moveViperToPosition(0);
        sleep(1000);

        whiteClaw.SuplexPixel();

        toSafety();
    }

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

    public void toSafety(){
        Action moveToSafety = drive.actionBuilder(drive.pose)
                .lineToY(-35.0)
                .strafeTo(new Vector2d(6, -36.0))
                .build();
        Actions.runBlocking(new ParallelAction(moveToSafety, whiteClaw.RetractArmAction()));
    }

    public void toSafetyf(){
        Action moveToSafety = drive.actionBuilder(drive.pose)
                .lineToY(-86.0)
                .strafeTo(new Vector2d(6, -86.0))
                .build();
        Actions.runBlocking(new ParallelAction(moveToSafety, whiteClaw.RetractArmAction()));
    }

    //to the spike mark
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

    //to the panel in the back
    public void toBackPanel(double targetX){

        Action moveRb3 = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineTo(new Vector2d(targetX,-38.5), Math.toRadians(-90))
                .build();
        Actions.runBlocking(moveRb3);
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
            if (spike == 1) {
                X = 12.5; Y = -3.0; ang = -32.0;
            } else if(spike == 2) {
                X = 21.5; Y = -6; ang = 0;
            } else {
                X = 28; Y = -3; ang = 0;
            }

            if (spike == 1 || spike == 2) {
                Action moveRb = drive.actionBuilder(drive.pose)
                        .splineTo(new Vector2d(X, Y), Math.toRadians(ang))
                        .build();
                Actions.runBlocking(new SequentialAction(moveRb, whiteClaw.PlacePixelAction()));

                //steps back from the spike mark
                Action moveBack = drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .splineTo(new Vector2d(11, -6), Math.toRadians(-90))
                        .build();
                Actions.runBlocking(new ParallelAction(moveBack, whiteClaw.RetractArmAction()));
            } else {
                // Spike 3 - Avoid Gates on Right
                Action moveThirdSMPlan = drive.actionBuilder(drive.pose)
                        .splineTo(new Vector2d(X, Y),Math.toRadians(ang))
                        .turn(Math.toRadians(90))
                        .build();
                Actions.runBlocking(new SequentialAction(moveThirdSMPlan, whiteClaw.PlacePixelAction()));
            }
        }
        drive.updatePoseEstimate();
    }

    private void toPixelStack() {
        if(propSpikeMark == 3){
            Action moveToStackThree = drive.actionBuilder(drive.pose)
                    .strafeTo(new Vector2d(40.5, PARAMS.toPixY))
                    .build();
            Actions.runBlocking(new SequentialAction(new ParallelAction(moveToStackThree, whiteClaw.RetractArmAction()),
                    whiteClaw.TopOfStackPickupAction(4)));
        }
        else{
            Action moveToStack = drive.actionBuilder(drive.pose)
                    .setReversed(false)
                    .splineTo(new Vector2d(28, 18), Math.toRadians(91))
                    .strafeTo(new Vector2d(40.5, PARAMS.toPixY))
                    .build();
            Actions.runBlocking(new SequentialAction( new ParallelAction(moveToStack, whiteClaw.RetractArmAction()),
                    whiteClaw.TopOfStackPickupAction(4) ));}
        whiteClaw.closeGrip();
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
    private void toBackPanel(int spikeMark){
        double targetX = 30;
        if (spikeMark == 3)
            targetX = 38;
        else if (spikeMark == 1)
            targetX = 25.0;

        Action moveRb3 = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineTo(new Vector2d(targetX,-39.0), Math.toRadians(-89))
                .build();
        Actions.runBlocking(moveRb3);
    }
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
        packet.put("Prop Mark", propSpikeMark);
        packet.put("Objects", tenFl.tlmObjectCnt);
        packet.put("Confidence", tenFl.tlmConfidence);
        packet.put("Obj X", tenFl.tlmBestPropXPos);
        packet.put("Obj Y", tenFl.tlmBestPropYPos);
        dashboard.sendTelemetryPacket(packet);
    }
}
