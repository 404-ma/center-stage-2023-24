package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helper.ClawMoves;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;



@Disabled
@Config
@Autonomous(name = "RR Auto Drive 4 - Box", group = "RoadRunner")
public class RRAutoDrive4 extends LinearOpMode {
    public static class Params {
        public double endX = 24;
        public double endY = 80;
        public double endHeading = 90;
    }

    public static Params PARAMS = new Params();

    @Override
    public void runOpMode() {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.addLine("RoadRunner Auto Drive 2 - Box");
        telemetry.addLine();
        telemetry.addData(">", "Press Start to Launch");
        telemetry.update();

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        ClawMoves whiteClaw = new ClawMoves(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;
        telemetry.clear();

        // Right Spike Mark
        whiteClaw.AutonomousStart();
        Action moveTwo = drive.actionBuilder(drive.pose)
                .splineTo( new Vector2d(18, 3), Math.toRadians(30))
                .build();
        Actions.runBlocking(new SequentialAction(moveTwo, whiteClaw.PlacePixelAction()));

        Action moveBack = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineTo( new Vector2d(6, 0), Math.toRadians(180))
                .build();
        Actions.runBlocking(moveBack);

        Action moveBar = drive.actionBuilder(drive.pose)
                .turnTo(Math.toRadians(-90))
                .lineToY(34)
                .build();
        Actions.runBlocking(new ParallelAction(moveBar, whiteClaw.RetractArmAction()));

        Action moveOne = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineTo( new Vector2d(PARAMS.endX,  PARAMS.endY), Math.toRadians(PARAMS.endHeading))
                .build();
        Actions.runBlocking(moveOne);
    }
}



