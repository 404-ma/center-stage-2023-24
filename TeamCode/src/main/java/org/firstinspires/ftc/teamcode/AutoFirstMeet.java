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
@Autonomous(name = "Autonomous", group = "1st meet")
public class AutoFirstMeet extends LinearOpMode {
    public static class Params {
        public double propSpikeMark = 1;
    }

    public static Params PARAMS = new Params();


    private FtcDashboard dashboard;
    private MecanumDrive drive;


    @Override
    public void runOpMode() {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.addLine("RoadRunner AutoFirstMeet");
        telemetry.addLine();
        telemetry.addData(">", "Press Start to Launch");
        telemetry.update();

        Servo flip = hardwareMap.servo.get("FlipServo");
        Servo grip = hardwareMap.servo.get("ClawServo");

        flip.setDirection(Servo.Direction.FORWARD);
        grip.setDirection(Servo.Direction.FORWARD);

        dashboard = FtcDashboard.getInstance();
        dashboard.clearTelemetry();
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();
        if (isStopRequested()) return;
        telemetry.clear();

        switch ((int) PARAMS.propSpikeMark) {
            case 3:
                // Left Spike Mark
                Action moveOne = drive.actionBuilder(drive.pose)
                        .splineTo(new Vector2d(17, -3), Math.toRadians(-27))
                        .build();
                Actions.runBlocking(moveOne);

                //PlacePixel(flip, grip);

                Action moveOneb = drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .splineTo(new Vector2d(6, 0), Math.toRadians(180))
                        .build();

                Actions.runBlocking(moveOneb);
                break;

            case 1:
                // Right Spike Mark
                Action moveTwo = drive.actionBuilder(drive.pose)
                        .splineTo(new Vector2d(18, 3), Math.toRadians(30))
                        .build();
                Actions.runBlocking(moveTwo);

                //PlacePixel(flip, grip);

                Action moveBack = drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .splineTo(new Vector2d(6, 0), Math.toRadians(180))
                        .build();

                Actions.runBlocking(moveBack);
                break;

            default:
                // Center Spike Mark
                Action moveThree = drive.actionBuilder(drive.pose)
                        .lineToX(21)
                        .build();
                Actions.runBlocking(moveThree);

                //PlacePixel(flip, grip);

                Action moveThreeb = drive.actionBuilder(drive.pose)
                        .lineToX(6)
                        .build();
                Actions.runBlocking(moveThreeb);
        }

        //Retract(flip, grip);

        Action moveBar = drive.actionBuilder(drive.pose)
                .turnTo(Math.toRadians(-90))
                .lineToY(34)
                .build();
        Actions.runBlocking(moveBar);

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
}






