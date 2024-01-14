package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Autonomous (name = "RR Auto Drive 3 - Spike Mark 2  ", group = "RoadRunner")
public class RRAutoDrive3 extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Load Introduction and Wait for Start
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.addLine("RoadRunner Auto Drive 3");
        telemetry.addLine();
        telemetry.addData(">", "Press Start to Launch");
        telemetry.update();

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();
        if (isStopRequested()) return;
        telemetry.clear();

        Action moveOne = drive.actionBuilder(drive.pose)
                .lineToX(25)
                .lineToX(7)
                .turnTo(Math.toRadians(-90))
                .lineToY(34)
                .turnTo(Math.toRadians(-100))
                .build();
        Actions.runBlocking(moveOne);

        sleep(1000);
    }

}

