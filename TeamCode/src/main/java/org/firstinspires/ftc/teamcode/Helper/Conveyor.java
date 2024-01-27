package org.firstinspires.ftc.teamcode.Helper;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Conveyor {

    public static class Params{

    }

    public static Params PARAMS = new Params();
    public DcMotorEx leftFront, leftBack, rightBack, rightFront;

    private Servo Conveyor;

    private Servo motor;

    public Encoder par0;

    public Conveyor(@NonNull HardwareMap hdwMap){
        leftFront = hdwMap.get(DcMotorEx.class, "frontLeft");
        leftBack = hdwMap.get(DcMotorEx.class, "backLeft");
        rightBack = hdwMap.get(DcMotorEx.class, "backRight");
        rightFront = hdwMap.get(DcMotorEx.class, "frontRight");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse motor directions if needed
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        par0 = new OverflowEncoder(new RawEncoder(hdwMap.get(DcMotorEx.class, "frontLeft")));
    }



}
