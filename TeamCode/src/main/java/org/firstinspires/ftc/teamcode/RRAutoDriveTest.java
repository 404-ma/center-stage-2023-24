package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Disabled
@Autonomous (name = "RR Auto Drive Test - Box", group = "RoadRunner")
public class RRAutoDriveTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Load Introduction and Wait for Start

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.addLine("RoadRunner Auto Drive Test - Box");
        telemetry.addLine();
        telemetry.addData(">", "Press Start to Launch");
        telemetry.update();

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();
        if (isStopRequested()) return;
        telemetry.clear();

        Action moveOne = drive.actionBuilder(drive.pose)
                .lineToX(40)
                .waitSeconds(1)
                .turnTo(Math.toRadians(90))
                .waitSeconds(1)
                .lineToY(30)
                .waitSeconds(1)
                .turnTo(Math.toRadians(180))
                .build();
        Actions.runBlocking(moveOne);

        sleep(1000);

        Action moveTwo = drive.actionBuilder(drive.pose)
                .lineToX(0)
                .waitSeconds(1)
                .turnTo(Math.toRadians(270))
                .waitSeconds(1)
                .lineToY(0)
                .waitSeconds(1)
                .turnTo(Math.toRadians(0))
                .waitSeconds(1)
                .build();
        Actions.runBlocking(moveTwo);
    }

}
