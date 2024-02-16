package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helper.ClawMoves;
import org.firstinspires.ftc.teamcode.Helper.Conveyor;
import org.firstinspires.ftc.teamcode.Helper.DistanceSystem;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.helper.TargetPose;

@Config
@Autonomous (name = "RR Auto Drive Blue", group = "RoadRunner")
public class Blue extends LinearOpMode {
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
        public double angleAtEnd = -90;
        public String versionNum = "3.1";

    }

    public static Params PARAMS = new Params();
    private FtcDashboard dashboard;
    private MecanumDrive drive;
    private ClawMoves whiteClaw;
    private Conveyor whiteConveyor;
    private DistanceSystem distSys;

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

        distSys = new DistanceSystem(hardwareMap);
        whiteClaw.AutonomousStart();


        waitForStart();
        if (isStopRequested()) return;
        telemetry.clear();


        switch((int) PARAMS.propSpikeMark){
            case 3:
                toSpikeMark(18.5,-3.0,-24, PARAMS.frontStage);
                if(PARAMS.frontStage){
                    toFrontPanel(36.5, PARAMS.partnerDead);
                }
                else{
                    toBackPanel(35.0);
                }
                break;
            case 1:
                toSpikeMark(19.5, 3.0, 32, PARAMS.frontStage);
                if(PARAMS.frontStage){
                    toFrontPanel(28.0, PARAMS.partnerDead);
                }
                else{
                    toBackPanel(28.0);
                }
                break;
            default:
                toSpikeMark(22.5, 3.2,0, PARAMS.frontStage);
                if(PARAMS.frontStage){
                    toFrontPanel(28.0, PARAMS.partnerDead);
                }
                else{
                    toBackPanel(28.0);
                }
                break;
        }
        //gets the position of the robot before dropping the pixel
        //SensorApproach();

        whiteClaw.PrepForPixel(false);

        whiteConveyor.moveViperToPosition(1400);
        whiteConveyor.moveConvForward();
        sleep(2000);
        whiteConveyor.stopConv();
        whiteConveyor.moveViperToPosition(0);
        sleep(1800);

        whiteClaw.SuplexPixel();
        secondHalf(PARAMS.angleAtEnd);

        //pick up
        whiteClaw.PrepForPixel(true);
        whiteClaw.closeGrip();
        whiteClaw.SuplexPixel();

        backSecondHalf();

        whiteClaw.PrepForPixel(false);
        whiteConveyor.moveViperToPosition(1400);
        whiteConveyor.moveConvForward();
        sleep(2000);
        whiteConveyor.stopConv();
        whiteConveyor.moveViperToPosition(0);
        sleep(1800);



    }

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
    public void toSpikeMark(double X, double Y, int ang, boolean position){
        double an;

        if (position) {
            an = -180;
        }
        else{
            an = 90;
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
                .splineTo(new Vector2d(targetX,38.5), Math.toRadians(90))
                .build();
        Actions.runBlocking(moveRb3);
    }

    public void secondHalf(double ang){
        Action moveSecHalf = drive.actionBuilder(drive.pose)
                //ending position y: -60
                //start off at 28, 38.5
                .splineTo(new Vector2d(9, -36), Math.toRadians(-90))
                .splineTo(new Vector2d(28, -60), Math.toRadians(-90))
                .build();
        Actions.runBlocking(new ParallelAction(moveSecHalf, whiteClaw.RetractArm()));
        }

    public void backSecondHalf(){
        Action moveBackSecHalf = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineTo(new Vector2d(9,12), Math.toRadians(90))
                .splineTo(new Vector2d(28,38.5), Math.toRadians(90)) //changes depending on the april tag & where the robot stars(front/back stage)
                .build();
        Actions.runBlocking(new ParallelAction(moveBackSecHalf, whiteClaw.RetractArm()));
    }
}