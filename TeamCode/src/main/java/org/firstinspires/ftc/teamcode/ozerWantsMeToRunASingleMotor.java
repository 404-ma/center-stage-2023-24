package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="OzerWantsMeToRunASingleMotor", group="Hardware")
public class ozerWantsMeToRunASingleMotor extends LinearOpMode {
    //balls :)
    @Override
    public void runOpMode() {
        DcMotor motor = hardwareMap.dcMotor.get("viperMotor");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (opModeIsActive()) {
            double power = gamepad1.right_stick_y * -1;
            int num = (int)power;
            motor.setPower(power);
            telemetry.addData("position", motor.getCurrentPosition());
            telemetry.addData("FOR OZER ELBEYLI", motor.getPower());
            telemetry.update();
        }
    }
}