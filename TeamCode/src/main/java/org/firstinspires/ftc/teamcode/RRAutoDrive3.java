package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
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
import org.firstinspires.ftc.teamcode.Helper.DrivetrainV2;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.helper.TargetPose;

@Config
@Autonomous (name = "RR Auto Drive 3 - Spike Marks", group = "RoadRunner")
public class RRAutoDrive3 extends LinearOpMode {
    /*
     *  FTC Dashboard Parameters
     */
    public static class Params {
        public double propSpikeMark = 1;    //  Which Spike Mark is the Prop Located on
        public boolean partnerDead = true;
        public boolean frontStage = true;
        public int dTime = 500;
        public double rangeNum = 2.0;
        public int rangeTime = 500;
        public double gainValueForward = 0.1;
        public double rangeValue = 2;
        public double gainValueRotation = 0.03;

    }

    public static Params PARAMS = new Params();
    private FtcDashboard dashboard;
    private MecanumDrive drive;
    private ClawMoves whiteClaw;
    private Conveyor whiteConveyor;
    private DistanceSystem distSys;
    private DrivetrainV2 drv;

    @Override
    public void runOpMode() {
        // Load Introduction and Wait for Start
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.addLine("RoadRunner Auto Drive 3");
        telemetry.addLine();
        telemetry.addData(">", "Press Start to Launch");
        telemetry.update();

        Servo arm = hardwareMap.servo.get("ArmServo");
        arm.setDirection(Servo.Direction.FORWARD);
        Servo flip = hardwareMap.servo.get("FlipServo");
        flip.setDirection(Servo.Direction.FORWARD);
        Servo grip = hardwareMap.servo.get("ClawServo");
        grip.setDirection(Servo.Direction.FORWARD);

        dashboard = FtcDashboard.getInstance();
        dashboard.clearTelemetry();
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        whiteClaw = new ClawMoves(hardwareMap);
        whiteConveyor = new Conveyor(hardwareMap);
        drv = new DrivetrainV2(hardwareMap);
        distSys = new DistanceSystem(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;
        telemetry.clear();
        whiteClaw.AutonomousStart();


        switch((int) PARAMS.propSpikeMark){
            case 3:
                toSpikeMark(17.0,-3.0,-24, PARAMS.frontStage);
                if(PARAMS.frontStage){
                    toFrontPanel(36.5, PARAMS.partnerDead);
                }
                else{
                    toBackPanel(35.0);
                }
                break;
            case 1:
                toSpikeMark(18.0, 3.0, 32, PARAMS.frontStage);
                if(PARAMS.frontStage){
                    toFrontPanel(28.0, PARAMS.partnerDead);
                }
                else{
                    toBackPanel(28.0);
                }
                break;
            default:
                toSpikeMark(21.0, -3.0,0, PARAMS.frontStage);
                if(PARAMS.frontStage){
                    toFrontPanel(30.0, PARAMS.partnerDead);
                }
                else{
                    toBackPanel(30.0);
                }
                break;
        }

        //gets the position of the robot before dropping the pixel
        drive.updatePoseEstimate();

        boolean first = true;
        TargetPose pose = distSys.getTargetPose(first);

   long timeout = System.currentTimeMillis()+PARAMS.rangeTime;

        while (pose.range > (pose.range-PARAMS.rangeValue) && System.currentTimeMillis() < timeout){
            first = false;

            double rangeError = (pose.range-PARAMS.rangeValue);

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            double forward = Range.clip(-rangeError * PARAMS.gainValueForward, -0.3, 0.3);
            double rotate = Range.clip(-pose.yaw * PARAMS.gainValueRotation, -0.25, 0.25);

            telemetry.addData("Distance", pose.range);
            telemetry.addData("Yaw", pose.yaw);
            telemetry.update();

            drv.setDriveVector(forward, 0, rotate);

            sleep(100);

        }

        whiteClaw.PrepForPixel();
        whiteConveyor.moveViper();
        sleep(1090);
        whiteConveyor.stopViper();
        whiteConveyor.moveConvForward();
        sleep(1000);
        whiteConveyor.stopConv();
        whiteConveyor.moveDownViper();
        sleep(1090);

    }
    //to the spike mark
    public void toSpikeMark(double X, double Y, int ang, boolean position){
        double an;

        if(position){
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
                    .splineTo(new Vector2d(targetX, 83), Math.toRadians(90))
                    .build();
            Actions.runBlocking(backdrop);
        }

        else {
            sleep(PARAMS.dTime);

            Action backdrop = drive.actionBuilder(drive.pose)
                    .setReversed(true)
                    .splineTo(new Vector2d(targetX, 83), Math.toRadians(90))
                    .build();
            Actions.runBlocking(backdrop);
        }
    }

    //to the panel in the back
    public void toBackPanel(double targetX){

        Action moveRb3 = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(targetX,36))
                .build();
        Actions.runBlocking(moveRb3);
    }
}