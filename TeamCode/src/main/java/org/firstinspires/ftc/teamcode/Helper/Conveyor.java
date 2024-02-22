package org.firstinspires.ftc.teamcode.Helper;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

@Config
public class Conveyor {

    public static class Params {
        public double conveyorSpeed = 0.90;
        public double viperSpeed = 0.5;
        public int viperMotorMaxPositionRelative = 3000;

    }

    public static Params PARAMS = new Params();
    public DcMotorEx viperMotor;
    private final CRServo conveyor;


    public Conveyor(@NonNull HardwareMap hdwMap) {
        viperMotor = hdwMap.get(DcMotorEx.class, "viperMotor");
        viperMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        viperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        conveyor = hdwMap.crservo.get("ConveyorServo");
    }

    public void moveConvForward() {
        conveyor.setPower(PARAMS.conveyorSpeed);
    }

    public void moveConvBackward() {
        conveyor.setPower(-PARAMS.conveyorSpeed);
    }

    public void moveConvPower( double power ) {
        conveyor.setPower(power);
    }

    public void stopConv() {
        conveyor.setPower(0);
    }

    public void moveViperToPosition(int position) {

        int absolutePosition = Range.clip(position, 0, PARAMS.viperMotorMaxPositionRelative);
        viperMotor.setTargetPosition(absolutePosition);
        viperMotor.setPower(PARAMS.viperSpeed);
        viperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void moveViperWithPower(double power, boolean override) {


        viperMotor.getMode();
        viperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (!override) {
            int viperPosition = viperMotor.getCurrentPosition();

            if (power > 0) {
                if (viperPosition >= PARAMS.viperMotorMaxPositionRelative)
                    power = 0;
                else if (viperPosition >= (PARAMS.viperMotorMaxPositionRelative * 0.95))
                    power = Math.min(power, 0.4);

            } else {
                if (viperPosition <= 0)
                    power = 0;
                else if (viperPosition <= (PARAMS.viperMotorMaxPositionRelative * 0.05))
                    power = Math.max(power, -0.4);
            }

            //30 inches - 3,000 tpi
            viperMotor.setPower(Range.clip(power, -1, 1));
        }
    }

}

