package org.firstinspires.ftc.teamcode.Helper;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@Config
public class Crane {
    public static class Params{
        public double craneSpeed = 0.9;
        public int craneMaxPositionRelative = 7900;
    }

    public static Params PARAMS = new Params();

    public DcMotorEx Crane;

    public Crane(@NonNull HardwareMap hdwMap){
        Crane = hdwMap.get(DcMotorEx.class, "craneMotor");
        Crane.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Crane.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void moveCraneToPosition(int position) {
        int checkedPosition = Range.clip(position, 0, PARAMS.craneMaxPositionRelative);
        Crane.setTargetPosition(checkedPosition);
        Crane.setPower(PARAMS.craneSpeed);
        Crane.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void moveCraneToRelativePosition(int relativePos) {
        int checkedPosition = Range.clip(relativePos + Crane.getCurrentPosition(), 0, PARAMS.craneMaxPositionRelative);
        Crane.setTargetPosition(checkedPosition);
        Crane.setPower(PARAMS.craneSpeed);
        Crane.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void moveCraneWithPower(double power, boolean override){
        Crane.getMode();
        Crane.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (!override) {
            int cranePosition = Crane.getCurrentPosition();
            if (power > 0) {
                if (cranePosition >= PARAMS.craneMaxPositionRelative)
                    power =0;
                else
                    power = Math.max(power, 0.4);
            } else {
                if (cranePosition <= 0)
                    power = 0;
                else
                    power = Math.max(power, -0.4);
            }
        }
        Crane.setPower(Range.clip(power, -1, 1));
    }
}
