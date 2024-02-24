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
        public boolean partnerDead = true;
        public boolean frontStage = false;
        public int dTime = 500;
        public double rangeNum = 2.0;
        public int rangeTime = 500;
        public double gainValueForward = 0.1;
        public double rangeValue = 2;
        public double gainValueRotation = 0.03;
        public double angleAtEnd = -90;
        public String versionNum = "3.2";
        public double propAng;
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
        telemetry.addData(">", "Press Start to Launch");
        telemetry.update();

        dashboard = FtcDashboard.getInstance();
        dashboard.clearTelemetry();

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        whiteClaw = new ClawMoves(hardwareMap);
        whiteConveyor = new Conveyor(hardwareMap);
        tenFl = new TensorFlow(hardwareMap);
        distSys = new DistanceSystem(hardwareMap);
        whiteClaw.AutonomousStart();

        waitForStart();
        telemetry.clear();
        if (isStopRequested()) return;

        propSpikeMark = tenFl.DetectProp(1000);
        updateTelemetry();
        tenFl.CleanUp();

        switch(propSpikeMark){
            case 3:
                PARAMS.propAng = 37.5;
                toSpikeMark(3, PARAMS.frontStage);
                if(PARAMS.frontStage){
                    toFrontPanel(PARAMS.propAng, PARAMS.partnerDead);
                }
                else{
                    toBackPanel(PARAMS.propAng);
                }
                break;
            case 1:
                PARAMS.propAng = 25.0;
                toSpikeMark(   1, PARAMS.frontStage);
                if(PARAMS.frontStage){
                    toFrontPanel(PARAMS.propAng, PARAMS.partnerDead);
                }
                else{
                    toBackPanel(PARAMS.propAng);
                }
                break;
            default:
                PARAMS.propAng = 29.0;
                toSpikeMark(2, PARAMS.frontStage);
                if(PARAMS.frontStage){
                    toFrontPanel(PARAMS.propAng, PARAMS.partnerDead);
                }
                else{
                    toBackPanel(PARAMS.propAng);
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
/*
        if(!PARAMS.frontStage){
            secondHalfBack();
        }
        else{
            BackMid();
        }

        //pick up
        whiteClaw.PrepForPixel(false);
        whiteClaw.moveLevel(5);
        whiteClaw.closeGrip();
        whiteClaw.SuplexPixel();

        /*
        if(!PARAMS.frontStage){
            backSecondHalfBack(PARAMS.propAng);
        }
        else{
            BackMidSec(PARAMS.propAng);
        }

        whiteClaw.PrepForPixel(false);
        whiteConveyor.moveViperToPosition(1400);
        sleep(1000);
        whiteConveyor.moveConvForward();
        sleep(2000);
        whiteConveyor.stopConv();
        whiteConveyor.moveViperToPosition(0);
        sleep(1800);
*/
    }

    //use sensor to square up to the panel
    private void SensorApproach() {
        long timeout = System.currentTimeMillis()+PARAMS.rangeTime;
        TargetPose pose = distSys.getTargetPose(true);  //Get Initial Values

        while (pose.range > (pose.range-PARAMS.rangeValue) && System.currentTimeMillis() < timeout){
            pose = distSys.getTargetPose(false);
            double rangeError = (pose.range-PARAMS.rangeValue);

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            double forward = Range.clip(-rangeError * PARAMS.gainValueForward, -0.3, 0.3);
            double rotate = Range.clip(-pose.yaw * PARAMS.gainValueRotation, -0.25, 0.25);

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(forward, 0), rotate));
            drive.updatePoseEstimate();
            sleep(30);
        }
    }

    //to the spike mark
    public void toSpikeMark(int spike, boolean position){
        double an;
        double X;
        double Y;
        double ang;

        if (position) {
            an = -180;
        }
        else{
            an = 90;
        }

        if(spike == 1){
            X = 13.5;
            Y = 3.0;
            ang = 32.0;
        }
        else if(spike == 2){
            X = 22.0;
            Y = 6.2;
            ang = 0.0;
        }
        else{
            X = 16;
            Y = 2;
            ang = 0;
        }

        //goes to specified spikeMark

        if(spike == 1 || spike == 2){
        Action moveRb = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(X, Y), Math.toRadians(ang))
                .build();
        Actions.runBlocking(new SequentialAction(moveRb, whiteClaw.PlacePixel()));}

        else {
                Action movethirdSMPlan = drive.actionBuilder(drive.pose)
                        .splineTo(new Vector2d(X, Y),Math.toRadians(ang)  )
                        .waitSeconds(2)
                        //.splineTo(new Vector2d(20, -4), Math.toRadians(-27))
                        .turnTo(-27)
                        .build();
                Actions.runBlocking(new SequentialAction(movethirdSMPlan, whiteClaw.PlacePixel()));
            }

        //steps back from the spike mark
        Action moveBack = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineTo(new Vector2d(11, 6), Math.toRadians(an))
                .build();
        Actions.runBlocking(new ParallelAction(moveBack, whiteClaw.RetractArm()));
    }

    //to the panel in the front
    public void toFrontPanel( double targetX, boolean partDead) {
        whiteClaw.RetractArm();

        Action moveBar = drive.actionBuilder(drive.pose)
                .turnTo(Math.toRadians(-90))
                .lineToY(40)
                .build();
        Actions.runBlocking(moveBar);

        if(partDead){
            Action backdrop = drive.actionBuilder(drive.pose)
                    .setReversed(true)
                    .splineTo(new Vector2d(targetX, 60), Math.toRadians(90))
                    .splineTo(new Vector2d(targetX, 86), Math.toRadians(90))
                    .build();
            Actions.runBlocking(backdrop);
        }

        else {
            sleep(PARAMS.dTime);

            Action backdrop = drive.actionBuilder(drive.pose)
                    .setReversed(true)
                    .splineTo(new Vector2d(targetX, 86), Math.toRadians(90))
                    .build();
            Actions.runBlocking(backdrop);
        }
    }

    //to the panel in the back
    public void toBackPanel(double targetX){

        Action moveRb3 = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineTo(new Vector2d(targetX,40.0), Math.toRadians(89))
                .build();
        Actions.runBlocking(moveRb3);
    }

    public void toSafety(){
        Action moveToSafety = drive.actionBuilder(drive.pose)
                .lineToY(35.0)
                .strafeTo(new Vector2d(6, 36.0))
                .build();
        Actions.runBlocking(new ParallelAction(moveToSafety, whiteClaw.RetractArm()));
    }

    //goes front to the pixels (when it started from backStage)
    public void secondHalfBack(){
        Action moveSecHalf = drive.actionBuilder(drive.pose)
                //ending position y: -60
                //start off at 28, 38.5
                .splineTo(new Vector2d(9, -36), Math.toRadians(-90))
                .splineTo(new Vector2d(28, -60), Math.toRadians(-90))
                .build();
        Actions.runBlocking(new ParallelAction(moveSecHalf, whiteClaw.RetractArm()));
        }
    //goes back to the panel (when it started from backStage)
    public void backSecondHalfBack(double ang){
        Action moveBackSecHalf = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineTo(new Vector2d(9,12), Math.toRadians(90))
                .splineTo(new Vector2d(ang,38.5), Math.toRadians(90)) //changes depending on the april tag & where the robot stars(front/back stage)
                .build();
        Actions.runBlocking(new ParallelAction(moveBackSecHalf, whiteClaw.RetractArm()));
    }

    //goes front to the pixel (when it started from the frontStage)
    public void secondHalfFront(){
        Action moveSecHalf = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(9, 0), Math.toRadians(-90))
                .splineTo(new Vector2d(28, -28), Math.toRadians(-90))
                .build();
        Actions.runBlocking(new ParallelAction(moveSecHalf, whiteClaw.RetractArm()));
    }

    //goes back to the panel (when it started from frontStage)
    public void backSecondHalfFront(double ang){
        Action moveBackSecHalf = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(9,60), Math.toRadians(90))
                .splineTo(new Vector2d(ang,86), Math.toRadians(90))
                .build();
        Actions.runBlocking(new ParallelAction(moveBackSecHalf, whiteClaw.RetractArm()));

    }

    //going through the middle gate (backStage)
    public void BackMid(){
//y = 38.5
        Action moveBackM = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineTo(new Vector2d(40,12), Math.toRadians(90))
                .splineTo(new Vector2d(40,-80), Math.toRadians(90))
                .build();
        Actions.runBlocking(new ParallelAction(moveBackM, whiteClaw.RetractArm()));

    }

    public void BackMidSec(double ang){
        Action moveBackMidSec = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(40, 12),Math.toRadians(90))
                .splineTo(new Vector2d(ang, 86), Math.toRadians(90))
                .build();
        Actions.runBlocking(new ParallelAction(moveBackMidSec, whiteClaw.RetractArm()));
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

}
