package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Autonomous(name="RoadRunner Drive Test", group = "RoadRunner")
public class RRDriveTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Load Introduction and Wait for Start
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.addLine("Road Runner Test Drive");
        telemetry.addLine();
        telemetry.addData(">", "Press Start to Launch");
        telemetry.update();

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();
        if (isStopRequested()) return;
        telemetry.clear();

        Action MoveOne = drive.actionBuilder(drive.pose)
                .lineToX(20)
                .waitSeconds(1)
                .turnTo(Math.toRadians(90))
                .waitSeconds(1)
                .lineToY(20)
                .waitSeconds(1)
                .turnTo(Math.toRadians(180))
                .build();

        Action MoveTwo = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(20, 20), Math.toRadians(90))
                .build();

        Actions.runBlocking(MoveOne);
        sleep(1000);
        Actions.runBlocking(MoveTwo);
    }

}
