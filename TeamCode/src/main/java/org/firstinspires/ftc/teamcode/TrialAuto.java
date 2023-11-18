package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Autonomous(name="TrialAuto")
public class TrialAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        double target = 90;
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");
        Servo grip = hardwareMap.servo.get("grip");
        DistanceSensor distance = hardwareMap.get(DistanceSensor.class, "distance");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters IMU;
        IMU = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        imu.initialize(IMU);
        imu.resetYaw();
        //frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //backLeft.setDirection(DcMotorSimple.Direction.REVERSE );
        while (opModeIsActive()){
            double Yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double shiftYaw = Yaw + 180;
            double shiftTarget = target + 180;
            double dist = distance.getDistance(DistanceUnit.CM);
            telemetry.addData("Yaw", Yaw);
            telemetry.addData("Distance", dist);
            double error = shiftTarget - shiftYaw;
            telemetry.addData("error",error);
            telemetry.update();


                if (error > target) {
                    frontLeft.setPower(error/10);
                    backLeft.setPower(error/10);
                    frontRight.setPower(-error/10);
                    backRight.setPower(-error/10);
                }
                if (error < target) {
                    frontLeft.setPower(-error/10);
                    backLeft.setPower(-error/10);
                    frontRight.setPower(error/10);
                    backRight.setPower(error/10);
                }
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
        }
    }
}
