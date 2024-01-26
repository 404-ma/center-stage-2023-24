package org.firstinspires.ftc.teamcode.Helper;

import static org.firstinspires.ftc.teamcode.Helper.gamePadInputV2.GameplayInputType.BUTTON_B;
import static org.firstinspires.ftc.teamcode.Helper.gamePadInputV2.GameplayInputType.BUTTON_X;
import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.security.PublicKey;


@Config
public class ClawMoves {
    // FTC Dashboard Parameters

    public static class Params {
        // TODO: Fill in Claw Servo Parameter
        public double armUpPos = 0.295;
        public double flipSuplexPos = 0.395;
        public double gripOpenPos = 0.28;
        public double gripClosedPos = 0.10;
        public double armDownPos = 0.2;
        public double flipDownPos = 0.54;
    }

    public static Params PARAMS = new Params();

    private double tlmGripPosition = 0;
    private Servo grip;
    private double tlmArmPosition = 0;
    private double tlmFlipPosition = 0;

    // Class Constructor
    public ClawMoves(@NonNull HardwareMap hdwMap) {
        // TODO: Add Servo Initialization
        Servo arm = hdwMap.servo.get("ArmServo");
        Servo flip = hdwMap.servo.get("FlipServo");
        Servo grip = hdwMap.servo.get("ClawServo");
    }


    // Claw Movements
    public void AutonomousStart (Servo arm,Servo flip, Servo grip) {
        // TODO: Fill in Code For Automous Start Position
        grip.setPosition(PARAMS.gripClosedPos);
        arm.setPosition(PARAMS.armUpPos);
        flip.setPosition(PARAMS.flipSuplexPos);
    }

    public void PlacePixel (Servo arm, Servo flip, Servo grip) {
        // TODO: Fill in Code to Drop Pixel
        // Reset Claw to Down and Open
        arm.setPosition(PARAMS.armDownPos);
        sleep(100);  // let Arm Move Away from Conveyor
        flip.setPosition(PARAMS.flipDownPos);
        sleep(500);
        grip.setPosition(PARAMS.gripOpenPos);
    }

    public void RetractArm (Servo arm, Servo flip, Servo grip) {
        // TODO: Fill in Code to Retract Arm w/o Pixel for Driving
        arm.setPosition(PARAMS.armUpPos);
        sleep(100);
        flip.setPosition(PARAMS.flipSuplexPos);
        // Wait for Suplex to Finish
        grip.setPosition(PARAMS.gripOpenPos);
    }

    public void SuplexPixel (Servo arm, Servo flip, Servo grip) {
        // TODO: Fill in Code to Close Grip, Pickup Pixel and Suplex it into Conveyor
        case BUTTON_X:
        // Pickup and Suplex Pixel
        tlmGripPosition = PARAMS.gripClosedPos;
        grip.setPosition(tlmGripPosition);
        sleep(300);  // Wait for Grip to Close
        tlmArmPosition = PARAMS.armUpPos;
        arm.setPosition(tlmArmPosition);
        sleep(100);
        tlmFlipPosition = PARAMS.flipSuplexPos;
        flip.setPosition(tlmFlipPosition);
        sleep(700);  // Wait for Suplex to Finish
        tlmGripPosition = PARAMS.gripOpenPos;
        grip.setPosition(tlmGripPosition);

    }

    public void PrepForPixel (Servo arm, Servo flip, Servo grip) {
        // TODO: Fill in Code to Prepare the Claw to Grab a Pixel.
        case BUTTON_B:
        // Reset Claw to Down and Open
        grip.setPosition(PARAMS.gripOpenPos);
        tlmArmPosition = PARAMS.armDownPos;
        arm.setPosition(tlmArmPosition);
        sleep(100);  // let Arm Move Away from Conveyor
        tlmFlipPosition = PARAMS.flipDownPos;
        flip.setPosition(tlmFlipPosition);
    }

}
