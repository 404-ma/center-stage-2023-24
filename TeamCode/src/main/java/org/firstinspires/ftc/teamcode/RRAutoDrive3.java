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
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

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
    }


    public static Params PARAMS = new Params();

    private FtcDashboard dashboard;
    private MecanumDrive drive;


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

        waitForStart();
        if (isStopRequested()) return;
        telemetry.clear();

        Start(arm,flip,grip);
        switch ((int) PARAMS.propSpikeMark) {
            case 3:
                // Left Spike Mark
                Action moveOne = drive.actionBuilder(drive.pose)
                        .splineTo( new Vector2d(17, -3), Math.toRadians(-27))
                        .build();
                Actions.runBlocking(moveOne);

                PlacePixel(arm, flip, grip);

                Action moveOneb = drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .splineTo(new Vector2d(6,0),Math.toRadians(180))
                        .build();

                Actions.runBlocking(moveOneb);
                break;

            case 1:
                // Right Spike Mark
                Action moveTwo = drive.actionBuilder(drive.pose)
                        .splineTo( new Vector2d(18, 3), Math.toRadians(30))
                        .build();
                Actions.runBlocking(moveTwo);

                PlacePixel(arm, flip, grip);
                Action moveBack = drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .splineTo( new Vector2d(6, 0), Math.toRadians(180))
                        .build();
                Actions.runBlocking(moveBack);

                break;

            default:
                // Center Spike Mark
                Action moveThree = drive.actionBuilder(drive.pose)
                        .lineToX(21)
                        .build();
                Actions.runBlocking(moveThree);

                PlacePixel(arm, flip, grip);

                Action moveThreeb = drive.actionBuilder(drive.pose)
                        .lineToX(6)
                        .build();
                Actions.runBlocking(moveThreeb);


        }
        Retract(arm,flip,grip);
        Action moveBar = drive.actionBuilder(drive.pose)
                .turnTo(Math.toRadians(-90))
                .lineToY(34)
                .build();
        Actions.runBlocking(moveBar);

        switch ((int)  PARAMS.propSpikeMark) {


           case 1:
               //Backdrop Position 1
               Action backdrop1 = drive.actionBuilder(drive.pose)
                       .setReversed(true)
                       .splineTo(new Vector2d(30,58), Math.toRadians(90))
                       .splineTo(new Vector2d(25.5,80), Math.toRadians(90))
                       .build();

               Actions.runBlocking(backdrop1);
               break;

           case 3:
               //Backdrop position 3
                   Action backdrop3 = drive.actionBuilder(drive.pose)
                   .setReversed(true)
                   .splineTo(new Vector2d(30,58), Math.toRadians(90))
                   .splineTo(new Vector2d(36.5,80), Math.toRadians(90))
                   .build();

               Actions.runBlocking(backdrop3);
               break;

           default:
               //Backdrop position 2
               Action backdrop2 = drive.actionBuilder(drive.pose)
                       .setReversed(true)
                       .splineTo(new Vector2d(30,58), Math.toRadians(90))
                       .splineTo(new Vector2d(30,80), Math.toRadians(90))
                       .build();

               Actions.runBlocking(backdrop2);
               break;


       }

    }


    public void PlacePixel(Servo arm, Servo flip, Servo grip){
        // Reset Claw to Down and Open
        arm.setPosition(PARAMS.armDownPos);
        sleep(100);  // let Arm Move Away from Conveyor
        flip.setPosition(PARAMS.flipDownPos);
        sleep(500);
        grip.setPosition(PARAMS.gripOpenPos);
    }

    public void Retract(Servo arm, Servo flip, Servo grip){
        arm.setPosition(PARAMS.armUpPos);
        sleep(100);
        flip.setPosition(PARAMS.flipSuplexPos);
        // Wait for Suplex to Finish
        grip.setPosition(PARAMS.gripOpenPos);
    }
    public void Start( Servo arm,Servo flip, Servo grip){
        grip.setPosition(PARAMS.gripClosedPos);
        arm.setPosition(PARAMS.armUpPos);
        flip.setPosition(PARAMS.flipSuplexPos);

        sleep(1000);


    }

}

