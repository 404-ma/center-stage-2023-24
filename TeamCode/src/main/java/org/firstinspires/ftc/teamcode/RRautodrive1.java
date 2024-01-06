package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helper.DistanceSystem;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Autonomous (name = "RR Auto Drive 1 - Strafe", group = "RoadRunner")
public class RRAutoDrive1 extends LinearOpMode {

        @Override
        public void runOpMode(){
                telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
                telemetry.addLine("RoadRunner Auto Drive 1 - Strafe");
                telemetry.addLine();
                telemetry.addData(">", "Press Start to Launch");
                telemetry.update();

                MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

                waitForStart();
                if (isStopRequested()) return;
                telemetry.clear();

                Action MoveOne = drive.actionBuilder(drive.pose)
                        .strafeTo( new Vector2d(30,30))
                        .build();

                while (opModeIsActive()) {
                        Actions.runBlocking(MoveOne);
                        sleep(1000);
                }
        }
}







