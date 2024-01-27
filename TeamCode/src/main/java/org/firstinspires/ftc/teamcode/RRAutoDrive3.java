package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helper.ClawMoves;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.RoboGoApril;

@Config
@Autonomous (name = "RR Auto Drive 3 - Spike Marks", group = "RoadRunner")
public class RRAutoDrive3 extends LinearOpMode {
    /*
     *  FTC Dashboard Parameters
     */
    public static class Params {
        public double propSpikeMark = 1;    //  Which Spike Mark is the Prop Located on
        public boolean partnerDead = false;
        public boolean backstage = true;
        public int dTime = 500;
    }

    public static Params PARAMS = new Params();
    private FtcDashboard dashboard;
    private MecanumDrive drive;
    private ClawMoves whiteClaw;

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

        waitForStart();
        if (isStopRequested()) return;
        telemetry.clear();

        whiteClaw.AutonomousStart();



        if (!PARAMS.backstage) {
            switch ((int) PARAMS.propSpikeMark) {
                case 3:
                    if(!PARAMS.partnerDead){
                        toSpikeMarkFrontND(17.0, -3.0, 25.5, -27);}
                    else{
                        toSpikeMarkFrontD(17.0, -3.0, 25.5, -27);
                        }
                    break;
                case 1:
                    if(!PARAMS.partnerDead) {
                        toSpikeMarkFrontND(18.0, 3.0, 36.5, 180);}
                    else{
                        toSpikeMarkFrontD(18.0, 3.0,36.5, 180);
                    }
                    break;
                default:
                    if(!PARAMS.partnerDead) {
                        toSpikeMarkFrontND(21.0, 0.0, 30.0, 30);}
                    else{
                        toSpikeMarkFrontD(21.0,0.0,30.0,30);
                    }
                    break;
                }
            }
        else {
            switch ((int) PARAMS.propSpikeMark) {
                case 3:
                    toSpikeMarkBack(17, -3, 25.5);
                    break;
                case 1:
                    toSpikeMarkBack(18, 3, 36.5);
                    break;
                default:
                    toSpikeMarkBack(21, 0, 30.0);
                    break;
            }
        }

    }

    public void toSpikeMarkFrontND(double X, double Y, double targetX, int ang){
        Action moveRb = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(X, Y), Math.toRadians(ang))
                .build();
        Actions.runBlocking(moveRb);

        whiteClaw.PlacePixel();

        Action moveBack = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineTo(new Vector2d(6, 0), Math.toRadians(180))
                .build();
        Actions.runBlocking(moveBack);

        whiteClaw.RetractArm();

        Action moveBar = drive.actionBuilder(drive.pose)
                .turnTo(Math.toRadians(-90))
                .lineToY(40)
                .build();
        Actions.runBlocking(moveBar);

        sleep(PARAMS.dTime);

        Action backdrop = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineTo(new Vector2d(targetX, 80), Math.toRadians(90))
                .build();
        Actions.runBlocking(backdrop);
    }

    public void toSpikeMarkFrontD(double X, Double Y, double targetX, int ang){

        Action moveRb = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(X, Y), Math.toRadians(ang))
                .build();
        Actions.runBlocking(moveRb);
        whiteClaw.PlacePixel();

        Action moveBack = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineTo(new Vector2d(6, 0), Math.toRadians(180))
                .build();
        Actions.runBlocking(moveBack);

        whiteClaw.RetractArm();

        Action moveBar = drive.actionBuilder(drive.pose)
                .turnTo(Math.toRadians(-90))
                .lineToY(40)
                .build();
        Actions.runBlocking(moveBar);

        Action backdrop = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineTo(new Vector2d(targetX, 60), Math.toRadians(90))
                .splineTo(new Vector2d(targetX, 80), Math.toRadians(90))
                .build();
        Actions.runBlocking(backdrop);
    }


    public void toSpikeMarkBack(double targetX, double targetY, double X){
        Action moveRb = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(targetX, targetY), Math.toRadians(-27))
                .build();
        Actions.runBlocking(moveRb);
        whiteClaw.PlacePixel();

        Action moveRb3 = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(6,0),Math.toRadians(90))
                .setReversed(true)
                .strafeTo(new Vector2d(30,10))
                .build();
        Actions.runBlocking(moveRb3);

        Action moveToBoard = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(X,10), Math.toRadians(90))
                .build();
        Actions.runBlocking(moveToBoard);

    }
}