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
        public double armForward = 1;
        public double armUpPos = 0.295;
        public double armDownPos = 0.2;
        public double flipForward = 1;
        public double flipDownPos = 0.54;
        public double flipSuplexPos = 0.395;
        public double gripForward = 1;
        public double gripOpenPos = 0.28;
        public double gripClosedPos = 0.10;
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
        switch ((int) PARAMS.propSpikeMark) {
            case 3:
                toSpikeMarkFS(17, -3);
                break;

            case 1:
                toSpikeMarkFS(18, 3);
                break;

            default:
                // Center Spike Mark
                Action moveThree = drive.actionBuilder(drive.pose)
                        .lineToX(21)
                        .build();
                Actions.runBlocking(moveThree);

                whiteClaw.PlacePixel();

                Action moveThreeb = drive.actionBuilder(drive.pose)
                        .lineToX(6)
                        .build();
                Actions.runBlocking(moveThreeb);
        }
        whiteClaw.RetractArm();

        Action moveBar = drive.actionBuilder(drive.pose)
                .turnTo(Math.toRadians(-90))
                .lineToY(40)
                .build();
        Actions.runBlocking(moveBar);

        if (PARAMS.partnerDead == false) {
            sleep(PARAMS.dTime);
            double targetX;
            double targetY = 80.0;

            if ((int) PARAMS.propSpikeMark == 1)
                targetX = 25.5;
            else if ((int) PARAMS.propSpikeMark == 3)
                targetX = 36.5;
            else
                targetX = 30.0;


            Action backdrop = drive.actionBuilder(drive.pose)
                    .setReversed(true)
                    .splineTo(new Vector2d(targetX, 60), Math.toRadians(90))
                    .splineTo(new Vector2d(targetX, targetY), Math.toRadians(90))
                    .build();

            Actions.runBlocking(backdrop);
        } else {
            double targetX;
            double targetY = 80.0;

            if ((int) PARAMS.propSpikeMark == 1)
                targetX = 25.5;
            else if ((int) PARAMS.propSpikeMark == 3)
                targetX = 36.5;
            else
                targetX = 30.0;


            Action backdrop = drive.actionBuilder(drive.pose)
                    .setReversed(true)
                    .splineTo(new Vector2d(targetX, targetY), Math.toRadians(90))
                    .build();

            Actions.runBlocking(backdrop);
        }

        if (PARAMS.backstage == false) {

            whiteClaw.AutonomousStart();
            switch ((int) PARAMS.propSpikeMark) {
                case 3:
                    toSpikeMarkFS(17, -3);
                    break;

                case 1:
                    toSpikeMarkFS(18, 3);
                    break;

                default:
                    // Center Spike Mark
                    Action moveThree = drive.actionBuilder(drive.pose)
                            .lineToX(21)
                            .build();
                    Actions.runBlocking(moveThree);

                    whiteClaw.PlacePixel();

                    Action moveThreeb = drive.actionBuilder(drive.pose)
                            .lineToX(6)
                            .build();
                    Actions.runBlocking(moveThreeb);
                }
                whiteClaw.RetractArm();

            }
    }

    public void toSpikeMarkBS(double targetX, double targetY){
        Action moveRb = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(targetX, targetY), Math.toRadians(-27))
                .build();
        Actions.runBlocking(moveRb);

        whiteClaw.PlacePixel();

        Action moveRb2 = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineTo(new Vector2d(6, 0), Math.toRadians(180))
                .build();
        Actions.runBlocking(moveRb2);
    }

    public void toSpikeMarkFS(double targetX,double targetY){
        Action moveRb = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(targetX, targetY), Math.toRadians(-27))
                .build();
        Actions.runBlocking(moveRb);

        whiteClaw.PlacePixel();

    }
}