package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Autonomous(name = "RR Auto Drive 4 - Box", group = "RoadRunner")
public class RRAutoDrive4 extends LinearOpMode {
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
        drive = new MecanumDrive(hardwareMap, new Pose2d(5.83, 33.7, 0));

        waitForStart();
        if (isStopRequested()) return;
        telemetry.clear();

        Action moveOne = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .lineToY(45)
                .waitSeconds(2)
                .splineTo( new Vector2d(23, 60), Math.toRadians(0))
                .build();
        Actions.runBlocking(moveOne);

    }


}





