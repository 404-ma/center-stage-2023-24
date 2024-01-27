package org.firstinspires.ftc.teamcode.Helper;


import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import android.os.SystemClock;


@Config
public class ClawMoves {
    // FTC Dashboard Parameters
    public static class Params {
        public double armUpPos = 0.295;
        public double flipSuplexPos = 0.395;
        public double gripOpenPos = 0.28;
        public double gripClosedPos = 0.10;

        public double armDownPos = 0.2;

        public double flipDownPos = 0.54;
    }

    public static Params PARAMS = new Params();

    private Servo arm;
    private Servo flip;
    private Servo grip;

    private int level;


    // Class Constructor
    public ClawMoves(@NonNull HardwareMap hdwMap) {
        arm = hdwMap.servo.get("ArmServo");
        flip = hdwMap.servo.get("FlipServo");
        grip = hdwMap.servo.get("ClawServo");
    }


    // Claw Movements
    public void AutonomousStart () {
        // Code For Automous Start Position
        grip.setPosition(PARAMS.gripClosedPos);
        arm.setPosition(PARAMS.armUpPos);
        flip.setPosition(PARAMS.flipSuplexPos);
    }

    public void PlacePixel () {
        // Reset Claw to Down and Open
        arm.setPosition(PARAMS.armDownPos);
        SystemClock.sleep(100); // let Arm Move Away from Conveyor
        flip.setPosition(PARAMS.flipDownPos);
        SystemClock.sleep(500);
        grip.setPosition(PARAMS.gripOpenPos);
    }

    public void RetractArm () {
        // Retract Arm w/o Pixel for Driving
        arm.setPosition(PARAMS.armUpPos);
        SystemClock.sleep(100);
        flip.setPosition(PARAMS.flipSuplexPos);
        // Wait for Suplex to Finish
        grip.setPosition(PARAMS.gripOpenPos);
    }

    public void closeGrip(){
        grip.setPosition(PARAMS.gripClosedPos);
        SystemClock.sleep(300); // Wait for Grip to Close
    }

    public void openGrip(){
        grip.setPosition(PARAMS.gripOpenPos);
        SystemClock.sleep(300); // Wait for Grip to Open
    }

    public void moveUp(int level){
        double pos = PARAMS.armDownPos + (level * 0.001);
        arm.setPosition(pos);
    }
    public void SuplexPixel () {
        // Pickup and Suplex Pixel
        arm.setPosition(PARAMS.armUpPos);
        SystemClock.sleep(100);
        flip.setPosition(PARAMS.flipSuplexPos);
        SystemClock.sleep(700); // Wait for Suplex to Finish
        grip.setPosition(PARAMS.gripOpenPos);
    }

    public void PrepForPixel () {
        // Reset Claw to Down and Open
        grip.setPosition(PARAMS.gripOpenPos);
        arm.setPosition(PARAMS.armDownPos);
        SystemClock.sleep(100); // let Arm Move Away from Conveyor
        flip.setPosition(PARAMS.flipDownPos);
    }

}