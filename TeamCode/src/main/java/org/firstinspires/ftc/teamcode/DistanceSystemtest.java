package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helper.DrivetrainV2;
import org.firstinspires.ftc.teamcode.Helper.DistanceSystem;
public class DistanceSystemtest extends LinearOpMode {
    private DrivetrainV2 drv;

    @Override
    public void runOpMode() {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.addLine("DistanceSystemtest");
        telemetry.addLine();
        telemetry.addData(">", "Press Start to Launch");
        telemetry.update();

        drv = new DrivetrainV2(hardwareMap);


        waitForStart();
        if (isStopRequested()) return;
        telemetry.clear();
        while (opModeIsActive()) {
            double distance = DistanceSystem.getDistance();
            double rangeError = (10 - distance);


            // Use the speed and turn "gains" to calculate how we want the robot to move.
            double forward = Range.clip(rangeError * 0.05, -0.3, 0.3);


            telemetry.addData("Distance", distance);
            telemetry.update();

            drv.setDriveVector(forward, 0, 0);

        }
    }
}
