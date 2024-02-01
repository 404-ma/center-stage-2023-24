package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helper.ClawMoves;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Autonomous (name = "RR Auto Drive 1 - Strafe", group = "RoadRunner")
public class RRautodrive1 extends LinearOpMode {

        private ClawMoves whiteClaw;

        @Override
        public void runOpMode(){
                telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
                telemetry.addLine("RoadRunner Auto Drive 1 - Strafe");
                telemetry.addLine();
                telemetry.addData(">", "Press Start to Launch");
                telemetry.update();

                whiteClaw = new ClawMoves(hardwareMap);

                MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

                waitForStart();
                telemetry.clear();

                while (opModeIsActive()) {
                        Actions.runBlocking(whiteClaw.PlacePixel());
                        sleep(500);
                        Actions.runBlocking(whiteClaw.RetractArm());

                }
        }
}








