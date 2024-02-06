package org.firstinspires.ftc.teamcode.Helper;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

@Config
public class Conveyor {

    public static class Params{
    }

    public static Params PARAMS = new Params();
    public DcMotorEx viperMotor;
    public Encoder viperEncoder;
    private CRServo conv;

    public Conveyor(@NonNull HardwareMap hdwMap){
         viperMotor =   hdwMap.get(DcMotorEx.class, "viperMotor");
         // TODO: Set ZeroPowerBehavior - Determine what action should be...
         // TODO: Add Odometry to Viper Motor
         viperEncoder = new OverflowEncoder(new RawEncoder(hdwMap.get(DcMotorEx.class, "viperMotor")));
         conv = hdwMap.crservo.get("conveyor");
    }

    public void moveConvForward(){

        conv.setPower(0.7);
    }

    public void moveConvBackward(){

        conv.setPower(-0.7);
    }
    public void stopConv(){

        conv.setPower(0);
    }

    public void moveViper(){

        viperMotor.setPower(0.5);
    }
    public void moveDownViper(){

        viperMotor.setPower(-0.5);
        // TODO:  Encoporate Encoder --viperEncoder.getPositionAndVelocity()
    }

    public void stopViper(){

        viperMotor.setPower(0);
    }

}
