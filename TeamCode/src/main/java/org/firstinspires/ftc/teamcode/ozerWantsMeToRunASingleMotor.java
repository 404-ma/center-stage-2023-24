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
        DcMotor frontLeft = hardwareMap.dcMotor.get("craneMotor");

        waitForStart();
        while (opModeIsActive()) {
            double power = gamepad1.right_stick_y;
            int num = (int)power;
            //frontLeft.setTargetPosition(num);
            //frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setPower(power);
            telemetry.addData("position", frontLeft.getCurrentPosition());
            telemetry.addData("FOR OZER ELBEYLI", frontLeft.getPower());
            telemetry.update();
        }
    }
}