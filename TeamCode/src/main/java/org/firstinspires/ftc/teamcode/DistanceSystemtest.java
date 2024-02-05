package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helper.DrivetrainV2;
import org.firstinspires.ftc.teamcode.Helper.DistanceSystem;
import org.firstinspires.ftc.teamcode.helper.TargetPose;

@Config
@TeleOp(name= "Distance System Test", group ="Test")
public class DistanceSystemtest extends LinearOpMode {

    public static class Params {
        public double gainValueForward = 0.1;
        public double rangeValue = 2;
        public double gainValueRotation = 0.03;
    }
    public static Params PARAMS = new Params();
    private FtcDashboard dashboard;
    private DrivetrainV2 drv;
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

        drv = new DrivetrainV2(hardwareMap);
        distSys = new DistanceSystem(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;
        telemetry.clear();
        boolean first = true;
        while (opModeIsActive()) {
            TargetPose pose = distSys.getTargetPose(first);
            first = false;

        double rangeError = (pose.range-PARAMS.rangeValue);

            // Use the speed and turn "gains" to calculate how we want the robot to move.
        double forward = Range.clip(-rangeError * PARAMS.gainValueForward, -0.3, 0.3);
            double rotate = Range.clip(-pose.yaw * PARAMS.gainValueRotation, -0.25, 0.25);


            telemetry.addData("Distance", pose.range);
            telemetry.addData("Yaw", pose.yaw);
            telemetry.update();

            drv.setDriveVector(forward, 0, rotate);



            sleep(100);

        }
        }
    }

