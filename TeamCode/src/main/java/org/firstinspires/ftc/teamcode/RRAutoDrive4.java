package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Config
@Autonomous(name = "RR Auto Drive 4 - Box", group = "RoadRunner")
public class RRAutoDrive4 extends LinearOpMode {
    public static class Params {
        public double endX = 18;
        public double endY = 60;
        public double endHeading = -90;
    }

    public static Params PARAMS = new Params();

    private FtcDashboard dashboard;
    private MecanumDrive drive;

    @Override
    public void runOpMode() {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.addLine("RoadRunner Auto Drive 2 - Box");
        telemetry.addLine();
        telemetry.addData(">", "Press Start to Launch");
        telemetry.update();

        dashboard = FtcDashboard.getInstance();
        dashboard.clearTelemetry();
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();
        if (isStopRequested()) return;
        telemetry.clear();

        // Right Spike Mark
        Action moveTwo = drive.actionBuilder(drive.pose)
                .splineTo( new Vector2d(18, 3), Math.toRadians(30))
                .setReversed(true)
                .splineTo( new Vector2d(6, 0), Math.toRadians(180))
                .build();
        Actions.runBlocking(moveTwo);

        Action moveBar = drive.actionBuilder(drive.pose)
                .turnTo(Math.toRadians(-90))
                .lineToY(34)
                .build();
        Actions.runBlocking(moveBar);

        Action moveOne = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineTo( new Vector2d(PARAMS.endX,  PARAMS.endY), Math.toRadians(PARAMS.endHeading))
                .build();
        Actions.runBlocking(moveOne);
    }


}





