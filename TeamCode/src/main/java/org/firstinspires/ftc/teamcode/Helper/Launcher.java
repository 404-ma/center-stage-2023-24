package org.firstinspires.ftc.teamcode.Helper;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Launcher {

    private Servo drone;

    public double position = 0;

    public Launcher(@NonNull HardwareMap hdwMap){
        Servo drone = hdwMap.servo.get("DroneServo");


    }

    public void startPosition(){
        drone.setDirection(Servo.Direction.FORWARD);
        position = 0.7;
        telemetry.addData("position:", position);
        telemetry.update();
        drone.setPosition(position);
    }

    public void fly(){
        drone.setDirection(Servo.Direction.FORWARD);
        position = 0.25;
        telemetry.addData("position:", position);
        telemetry.update();
        drone.setPosition(position);
    }

}
