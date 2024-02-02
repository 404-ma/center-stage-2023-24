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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helper.ClawMoves;
import org.firstinspires.ftc.teamcode.Helper.Conveyor;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Config
@Autonomous (name = "RR Auto Drive 3 - Spike Marks", group = "RoadRunner")
public class RRAutoDrive3 extends LinearOpMode {
    /*
     *  FTC Dashboard Parameters
     */
    public static class Params {
        public double propSpikeMark = 1;    //  Which Spike Mark is the Prop Located on
        public boolean partnerDead = true;
        public boolean backstage = true;
        public int dTime = 500;
    }

    public static Params PARAMS = new Params();
    private FtcDashboard dashboard;
    private MecanumDrive drive;
    private ClawMoves whiteClaw;
    private Conveyor whiteConveyor;



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

        waitForStart();
        if (isStopRequested()) return;
        telemetry.clear();
        whiteClaw.AutonomousStart();

        if (!PARAMS.backstage) {
            switch ((int) PARAMS.propSpikeMark) {
                case 3:
                    toSpikeMarkFront(13.0, -3.0, 36.5, -24, PARAMS.partnerDead);
                    break;
                case 1:
                    toSpikeMarkFront(13.0, 3.0, 25.5, 32, PARAMS.partnerDead);
                    break;
                default:
                    toSpikeMarkFront(21.0, -3.0, 30.0, 0, PARAMS.partnerDead);
                    break;
            }
        } else {
            switch ((int) PARAMS.propSpikeMark) {
                case 3:
                    toSpikeMarkBack(13.0, -3.0, 35.0, -24);
                    break;
                case 1:
                    toSpikeMarkBack(13.0, 3.0, 25.5, 32);
                    break;
                default:
                    toSpikeMarkBack(21.0, -3.0, 30.0, 0);
                    break;
            }
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

    public void toSpikeMarkFront(double X, double Y, double targetX, int ang, boolean partDead) {
        Action moveRb = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(X, Y), Math.toRadians(ang))
                .build();
        Actions.runBlocking(new SequentialAction(moveRb, whiteClaw.PlacePixel()));

        Action moveBack = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineTo(new Vector2d(6, 0), Math.toRadians(180))
                .build();
        Actions.runBlocking(new ParallelAction(moveBack, whiteClaw.RetractArm()));

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

    public void toSpikeMarkBack(double X, double Y, double targetX, int ang){
        Action moveRb = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(X, Y), Math.toRadians(ang))
                .build();
        Actions.runBlocking(new SequentialAction(moveRb, whiteClaw.PlacePixel()));

        Action moveBack = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineTo(new Vector2d(6, 0), Math.toRadians(90))
                .build();
        Actions.runBlocking(new ParallelAction(moveBack, whiteClaw.RetractArm()));

        sleep(850);

        Action moveRb3 = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(targetX,36))
                .build();
        Actions.runBlocking(moveRb3);

    }
}