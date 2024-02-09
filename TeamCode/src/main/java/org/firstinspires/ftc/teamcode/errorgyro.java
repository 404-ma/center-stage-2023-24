package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// TODO: Can we Delete This Code

@TeleOp
public class errorgyro extends LinearOpMode {
    //balls :)
    @Override
    public void runOpMode() {
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");
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

        double targetD = 20;
        double targetG = 20;
        double initError = 0;
        double kp = 0.05;
        double kd = 0.2;
        double ki = 0.0001;
        double errorSum = 0;

        waitForStart();
        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            double newError = targetD - distance.getDistance(DistanceUnit.CM);
            double kalError = (newError+initError)/2;
            errorSum = kalError + errorSum;
            double diffError = kalError-initError;
            double power = -kp*kalError-kd*diffError-ki*errorSum;
            initError = kalError;
            if (power > 0.5){
                power = 0.5;
            }
            if(power < -0.5){
                power = -0.5;
            }

            double Yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double dist = distance.getDistance(DistanceUnit.CM);
            telemetry.addData("Yaw", Yaw);
            telemetry.addData("Distance", dist);
            telemetry.update();
            double error = (targetG - Yaw)/4;

            if (error > 0) {
                frontRight.setPower(power + frontRightPower + error);
                frontLeft.setPower(power + frontLeftPower - error);
                backRight.setPower(power + backRightPower + error);
                backLeft.setPower(power + backLeftPower - error);
            }
            if (error < 0) {
                frontRight.setPower(power + frontRightPower - error);
                frontLeft.setPower(power + frontLeftPower + error);
                backRight.setPower(power + backRightPower - error);
                backLeft.setPower(power + backLeftPower + error);
            }
        }
    }
}