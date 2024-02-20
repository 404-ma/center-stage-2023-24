package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
@Disabled
public class error extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");
        Servo grip = hardwareMap.servo.get("grip");
        DistanceSensor distance = hardwareMap.get(DistanceSensor.class, "distance");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE );

        double target = 20;
        double initError = 0;
        double kp = 0.05;
        double kd = 0.2;
        double ki = 0.0001;
        double errorSum = 0;
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double newError = target - distance.getDistance(DistanceUnit.CM);
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

            frontLeft.setPower(power);
            backLeft.setPower(power);
            frontRight.setPower(power);
            backRight.setPower(power);

        }

    }
}