package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Config
@Autonomous (name = "RR Auto Drive 3 - Spike Marks", group = "RoadRunner")
public class RRAutoDrive3 extends LinearOpMode {
    /*
     *  FTC Dashboard Parameters
     */
    public static class Params {
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
                        .splineTo( new Vector2d(17, -3), Math.toRadians(-27))
                        .setReversed(true)
                        .splineTo(new Vector2d(6,0),Math.toRadians(180))
                        .build();
                Actions.runBlocking(moveOne);
                break;

            case 1:
                // Right Spike Mark
                Action moveTwo = drive.actionBuilder(drive.pose)
                        .splineTo( new Vector2d(18, 3), Math.toRadians(30))
                        .setReversed(true)
                        .splineTo( new Vector2d(6, 0), Math.toRadians(180))
                        .build();
                Actions.runBlocking(moveTwo);
                break;

            default:
                // Center Spike Mark
                Action moveThree = drive.actionBuilder(drive.pose)
                        .lineToX(21)
                        .lineToX(6)
                        .build();

                Actions.runBlocking(moveThree);

        }

            Action moveBar = drive.actionBuilder(drive.pose)
                    .turnTo(Math.toRadians(-90))
                    .lineToY(34)
                    .turnTo(Math.toRadians(-100))
                    .build();
            Actions.runBlocking(moveBar);



        sleep(1000);
    }

}

