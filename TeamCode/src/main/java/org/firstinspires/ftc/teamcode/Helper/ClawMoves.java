package org.firstinspires.ftc.teamcode.Helper;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.security.PublicKey;


@Config
public class ClawMoves {
    // FTC Dashboard Parameters

    public static class Params {
        // TODO: Fill in Claw Servo Parameter
        public double temp = 1;
    }

    public static Params PARAMS = new Params();



    // Class Constructor
    public ClawMoves(@NonNull HardwareMap hdwMap) {
        // TODO: Add Servo Initialization
    }


    // Claw Movements
    public void AutonomousStart () {
        // TODO: Fill in Code For Automous Start Position
    }

    public void PlacePixel () {
        // TODO: Fill in Code to Drop Pixel
    }

    public void RetractArm () {
        // TODO: Fill in Code to Retract Arm w/o Pixel for Driving
    }

    public void SuplexPixel () {
        // TODO: Fill in Code to Close Grip, Pickup Pixel and Suplex it into Conveyor
    }

    public void PrepForPixel () {
        // TODO: Fill in Code to Prepare the Claw to Grab a Pixel.
    }

}
