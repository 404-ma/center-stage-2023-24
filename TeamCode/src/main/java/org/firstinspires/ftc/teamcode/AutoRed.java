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
        public double propSpikeMark = 2;    //  Which Spike Mark is the Prop Located on
        public boolean partnerDead = true;
        public boolean frontStage = false;
        public int dTime = 500;
        public double rangeNum = 2.0;
        public int rangeTime = 500;
        public double gainValueForward = 0.1;
        public double rangeValue = 2;
        public double gainValueRotation = 0.03;
        public String versionNum = "3.2";

    }

    public static Params PARAMS = new Params();
    private FtcDashboard dashboard;
    private MecanumDrive drive;
    private ClawMoves whiteClaw;
    private Conveyor whiteConveyor;
    private DistanceSystem distSys;
    private DrivetrainV2 drv;
    private TensorFlow tenFl;
    public int spikeMark = 0;

    @Override
    public void runOpMode() {
        // Load Introduction and Wait for Start
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.addLine("RoadRunner Auto Drive RED");
        telemetry.addLine();
        telemetry.addLine().addData("Version", PARAMS.versionNum);
        telemetry.addLine();
        telemetry.addData(">", "Press Start to Launch");
        telemetry.update();

//
        dashboard = FtcDashboard.getInstance();
        dashboard.clearTelemetry();
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        whiteClaw = new ClawMoves(hardwareMap);
        whiteConveyor = new Conveyor(hardwareMap);
        tenFl = new TensorFlow(hardwareMap);
        distSys = new DistanceSystem(hardwareMap);
        whiteClaw.AutonomousStart();

        waitForStart();
        if (isStopRequested()) return;

        spikeMark = tenFl.DetectProp(1500);
        telemetry.clear();
        tenFl.CleanUp();

        switch((int) spikeMark){
            case 3:
                toSpikeMark(20.5,4.0,27, PARAMS.frontStage);
                if(PARAMS.frontStage){
                    toFrontPanel(36.5, PARAMS.partnerDead);
                }
                else{
                    toBackPanel(35.0);
                }
                break;
            case 1:
                toSpikeMark(18.5, -3.0, -30, PARAMS.frontStage);
                if(PARAMS.frontStage){
                    toFrontPanel(27.0, PARAMS.partnerDead);
                }
                else{
                    toBackPanel(27.0);
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

        whiteConveyor.moveViperToPosition(1400);
        sleep(1000);
        whiteConveyor.moveConvForward();
        sleep(2000);
        whiteConveyor.stopConv();
        whiteConveyor.moveViperToPosition(0);
        sleep(1800);
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
        Actions.runBlocking(new SequentialAction(moveRb, whiteClaw.PlacePixel()));

        if(!PARAMS.frontStage){
            sleep(850);
            whiteClaw.RetractArm();
        }

        Action moveBack = drive.actionBuilder(drive.pose)

                .setReversed(true)
                .splineTo(new Vector2d(6, 0), Math.toRadians(an))
                .build();
        Actions.runBlocking(new ParallelAction(moveBack, whiteClaw.RetractArm()));
    }

    //to the panel in the front
    public void toFrontPanel( double targetX, boolean partDead) {

        whiteClaw.RetractArm();

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


    private void updateTelemetry() {
        telemetry.addLine("TensorFlow");
        telemetry.addLine().addData("Prop Mark", spikeMark );
        telemetry.addLine().addData("Objects", tenFl.tlmObjectCnt);
        telemetry.addLine().addData("Confidence", tenFl.tlmConfidence);
        telemetry.addLine().addData("Obj X", tenFl.tlmBestPropXPos);
        telemetry.addLine().addData("Obj Y", tenFl.tlmBestPropYPos);
        telemetry.update();

        // FTC Dashboard Telemetry
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Prop Mark", spikeMark);
        packet.put("Objects", tenFl.tlmObjectCnt);
        packet.put("Confidence", tenFl.tlmConfidence);
        packet.put("Obj X", tenFl.tlmBestPropXPos);
        packet.put("Obj Y", tenFl.tlmBestPropYPos);
        dashboard.sendTelemetryPacket(packet);
    }
}
