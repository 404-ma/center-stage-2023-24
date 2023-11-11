package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@TeleOp
public class twoSensorPID extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");
        DistanceSensor distanceR = hardwareMap.get(DistanceSensor.class, "distanceR");
        DistanceSensor distanceL = hardwareMap.get(DistanceSensor.class, "distanceL");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE );
        double initErrorDist = 0;
        double initErrorRot = 0;
        double lastErrorDist = 0;
        double lastErrorRot = 0;


        double targetDist = 10;
        double approachSpeedLimDist = 0.25;
        double approachSpeedLimRot = 0.1;
        double lastTargetDist = targetDist;

        double integralSumDist = 0;
        double integralSumRot = 0;
        double maxRotSum = 60;
        double maxDistSum = 60;

        double kalDist = 0.9;
        double kalRot = 0.7;
        double previousFilterEstimateDist = 0;
        double currentFilterEstimateDist = 0;
        double previousFilterEstimateRot = 0;
        double currentFilterEstimateRot = 0;
        double distRCMFil=0;
        double distLCMFil=0;

        double rotDist = 30;

        double powerRot = 0;
        double powerDist = 0;

        double kpRot = 0.04;
        double kdRot = 0;
        double kiRot = 0;
        double kpDist = 0.042;
        double kdDist = 0.0029*.75;
        double kiDist = 0.15/20;

        ElapsedTime timer = new ElapsedTime();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double distRCM = distanceR.getDistance(DistanceUnit.CM);
            double distLCM = distanceL.getDistance(DistanceUnit.CM);

            //double distRCMFilt=




            double errorDist = ((distLCM + distRCM)/2)-targetDist;

            double errorChangeDist = errorDist - lastErrorDist;
            currentFilterEstimateDist = (kalDist*previousFilterEstimateDist)+(1-kalDist)*errorChangeDist;
            previousFilterEstimateDist = currentFilterEstimateDist;
            double derivativeDist = currentFilterEstimateDist/timer.seconds();
            integralSumDist = integralSumDist + (errorDist*timer.seconds());
            if (integralSumDist > maxDistSum){
                integralSumDist = maxDistSum;
            }
            if (integralSumDist < -maxDistSum){
                integralSumDist = -maxDistSum;
            }
            if (targetDist != lastTargetDist) {
                integralSumDist = 0;
            }
            if (gamepad1.a) {
                powerDist = (kpDist * errorDist) + (kdDist * derivativeDist) + (kiDist * integralSumDist);
            }
            else {
                powerDist = 0;
            }

            if (powerDist > approachSpeedLimDist){
                powerDist = approachSpeedLimDist;
            }
            if (powerDist < -approachSpeedLimDist){
                powerDist = -approachSpeedLimDist;
            }



            double errorRot = distLCM - distRCM;

            double errorChangeRot = errorRot - lastErrorRot;
            currentFilterEstimateRot = (kalRot*previousFilterEstimateRot)+(1-kalRot)*errorChangeRot;
            previousFilterEstimateRot = currentFilterEstimateRot;
            double derivativeRot = currentFilterEstimateRot/timer.seconds();
            integralSumRot = integralSumRot + (errorRot*timer.seconds());
            if (integralSumRot > maxRotSum){
                integralSumRot = maxRotSum;
            }
            if (integralSumRot < -maxRotSum){
                integralSumRot = -maxRotSum;
            }

            if(Math.abs(errorDist) < rotDist && gamepad1.a){
                powerRot = (kpRot*errorRot)+(kdRot*derivativeRot)+(kiRot*integralSumRot);
            }
            else{
                powerRot = 0;
            }

            if (powerRot > approachSpeedLimRot){
                powerRot = approachSpeedLimRot;
            }
            if (powerRot < -approachSpeedLimRot){
                powerRot = -approachSpeedLimRot;
            }

            double y = gamepad1.left_stick_y*.5;
            double x = gamepad1.left_stick_x * 1.1*.5;
            double rx = gamepad1.right_stick_x*.5;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;


            frontLeft.setPower(-powerRot + powerDist - frontLeftPower);
            backLeft.setPower(-powerRot + powerDist - backLeftPower);
            frontRight.setPower(powerRot + powerDist - frontRightPower);
            backRight.setPower(powerRot + powerDist - backRightPower);

            telemetry.addData("Right Sensor Distance", distanceR.getDistance(DistanceUnit.CM));
            telemetry.addData("Left Sensor Distance", distanceL.getDistance(DistanceUnit.CM));
            telemetry.addData("integralSumDist", integralSumDist);
            telemetry.addData("derivativeDist", derivativeDist);
            telemetry.addData("timer", timer.seconds());
            telemetry.update();

            lastErrorDist = errorDist;
            lastTargetDist = targetDist;
            timer.reset();
            //if (error > 0.2){

            //} else if (error < -0.2){
            // frontLeft.setPower(rotationalSpeed);
            //backLeft.setPower(rotationalSpeed);
            //frontRight.setPower(-rotationalSpeed);
            //backRight.setPower(-rotationalSpeed);
            //}

        }

    }
}