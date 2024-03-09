package org.firstinspires.ftc.teamcode.Helper;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Launcher {
    public static class Params {
        public double launchPos = 0.25;
        public double startPos = 0.73;
    }

    public static Params PARAMS = new Params();

    private final Servo drone;

    public double tlmPosition = 0;

    public Launcher(@NonNull HardwareMap hdwMap){
        drone = hdwMap.servo.get("DroneServo");
        drone.setDirection(Servo.Direction.FORWARD);
    }

    public void startPosition(){
        tlmPosition = PARAMS.startPos;
        drone.setPosition(tlmPosition);
    }

    public void fly(){
        tlmPosition = PARAMS.launchPos;
        drone.setPosition(tlmPosition);
    }
}
