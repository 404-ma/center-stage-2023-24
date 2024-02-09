package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helper.DistanceSystem;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.helper.TargetPose;

import java.util.Locale;

@Config
@TeleOp(name= "Distance System Test", group ="Test")
public class DistanceSystemtest extends LinearOpMode {

    public static class Params {
        public double gainValueForward = 0.1;
        public double rangeValue = 2;
        public int rangeTime = 1000;
        public double gainValueRotation = 0.03;
    }

    public static Params PARAMS = new Params();

    private FtcDashboard dashboard;
    private MecanumDrive drive;
    private DistanceSystem distSys;

    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        dashboard.clearTelemetry();

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.addLine("Distance System Test");
        telemetry.addLine();
        telemetry.addData(">", "Press Start to Launch");
        telemetry.update();

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        distSys = new DistanceSystem(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;
        telemetry.clear();

        long timeout = System.currentTimeMillis() + PARAMS.rangeTime;
        TargetPose pose = distSys.getTargetPose(true);  //Get Initial Values

        while (pose.range > (pose.range - PARAMS.rangeValue) && System.currentTimeMillis() < timeout) {
            pose = distSys.getTargetPose(false);

            SensorApproach(pose);
            update_telemetry(pose);
            sleep(100);
        }
    }

    private void SensorApproach(TargetPose target) {
        double rangeError = (target.range - PARAMS.rangeValue);

        // Use the speed and turn "gains" to calculate how we want the robot to move.
        double forward = Range.clip(-rangeError * PARAMS.gainValueForward, -0.3, 0.3);
        double rotate = Range.clip(-target.yaw * PARAMS.gainValueRotation, -0.25, 0.25);

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(forward, 0), rotate));
        drive.updatePoseEstimate();
    }

    private void update_telemetry(TargetPose target) {
        telemetry.addLine("Distance Sensors");
        String dsTime = new java.text.SimpleDateFormat("yyyy.MM.dd HH:mm:ss.SSS", Locale.US).format(distSys.tlm_LastCheckedTimestamp);
        telemetry.addLine().addData( "Timestamp", dsTime);
        telemetry.addLine().addData( "Left     ", distSys.tlm_LeftDistance);
        telemetry.addLine().addData( "Right    ", distSys.tlm_RightDistance);
        telemetry.addLine().addData( "Rotation ", distSys.tlm_CurrentRotation);
        telemetry.addLine().addData( "P Range  ", distSys.tlm_LastPose.range);
        telemetry.addLine().addData( "P Bearing", distSys.tlm_LastPose.bearing);
        telemetry.addLine().addData( "P Yaw    ", distSys.tlm_LastPose.yaw);

        telemetry.addData("Distance", target.range);
        telemetry.addData("Yaw", target.yaw);
        telemetry.update();

        // FTC Dashboard Telemetry
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Distance Goal", PARAMS.rangeValue);
        packet.put("Distance Error", (distSys.tlm_LastPose.range - PARAMS.rangeValue));
        packet.put("Pose Range", distSys.tlm_LastPose.range);
        packet.put("Pose Yaw", distSys.tlm_LastPose.yaw);
        packet.put("Sensor L",  distSys.tlm_LeftDistance);
        packet.put("Sensor R",  distSys.tlm_RightDistance);
        dashboard.sendTelemetryPacket(packet);
    }
}
