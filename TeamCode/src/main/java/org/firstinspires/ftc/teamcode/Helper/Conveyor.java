package org.firstinspires.ftc.teamcode.Helper;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

@Config
public class Conveyor {

    public static class Params {

        public double conveyorSpeed = 0.7;
        public double viperSpeed = 0.5;
        public int viperMotorMaxPositionRelative = 3000;

    }

    public static Params PARAMS = new Params();
    public DcMotorEx viperMotor;
    public Encoder viperEncoder;
    private CRServo conv;


    private int viperMotorStart;

    public Conveyor(@NonNull HardwareMap hdwMap) {
        viperMotor = hdwMap.get(DcMotorEx.class, "viperMotor");
        viperMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        conv = hdwMap.crservo.get("conveyor");
        viperMotorStart = viperMotor.getCurrentPosition();
    }

    public void moveConvForward() {
        conv.setPower(0.7);
    }

    public void moveConvBackward() {
        conv.setPower(-0.7);
    }

    public void stopConv() {

        conv.setPower(0);
    }

    public void moveViperToPosition(int position) {
        int absolutePosition = viperMotorStart + Range.clip(position, 0, PARAMS.viperMotorMaxPositionRelative);
        viperMotor.setTargetPosition(absolutePosition);
        viperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void moveViperWithPower(int power){
        int viperPosition = viperMotor.getCurrentPosition();
        if ((viperPosition <= viperMotorStart) | (viperPosition >= viperMotorStart +PARAMS.viperMotorMaxPositionRelative))
            viperMotor.setPower(0);
        else {
            viperMotor.setPower( Range.clip(power,-1,1));
        }
    }



}
