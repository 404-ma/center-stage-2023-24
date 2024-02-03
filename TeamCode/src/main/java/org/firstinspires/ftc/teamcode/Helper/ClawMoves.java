package org.firstinspires.ftc.teamcode.Helper;

import static org.firstinspires.ftc.teamcode.Helper.DeferredActions.DeferredActionType;


import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import android.os.SystemClock;


@Config
public class ClawMoves {
    // FTC Dashboard Parameters
    public static class Params {
        public double armUpPos = 0.287;
        public double armDownPos = 0.215;

        public double flipSuplexPos = 0.395;
        public double flipDownPos = 0.537;

        public double gripOpenPos = 0.655;
        public double gripClosedPos = 0.380;
    }

    public static Params PARAMS = new Params();

    public double tlmArmPosition = -1;
    public double tlmGripPosition = -1;
    public double tlmFlipPosition = -1;

    private Servo arm;
    private Servo flip;
    private Servo grip;



    // Class Constructor
    public ClawMoves(@NonNull HardwareMap hdwMap) {
        arm = hdwMap.servo.get("ArmServo");
        arm.setDirection(Servo.Direction.FORWARD);

        flip = hdwMap.servo.get("FlipServo");
        flip.setDirection(Servo.Direction.FORWARD);

        grip = hdwMap.servo.get("ClawServo");
        grip.setDirection(Servo.Direction.FORWARD);
    }

     // Single Servo Movements
    public void MoveArm(double position) {
        arm.setPosition(position);
        tlmArmPosition = position;
    }

    public void MoveFlip(double position) {
        flip.setPosition(position);
        tlmFlipPosition = position;
    }

    public void MoveGrip(double position) {
        grip.setPosition(position);
        tlmGripPosition = position;
    }

    /*
     * Autonomous Claw Movements
     */
    public void AutonomousStart () {
        // Code For Autonomous Start Position
        MoveGrip(PARAMS.gripClosedPos);
        MoveArm(PARAMS.armUpPos);
        MoveFlip(PARAMS.flipSuplexPos);
    }


    public Action PlacePixel() {
        return packet -> {
            // Autonomous Place Preloaded Pixel on Mat
            MoveArm(PARAMS.armDownPos);
            // Allow Time for Arm to Move Away from Conveyor
            SystemClock.sleep(100);
            MoveFlip(PARAMS.flipDownPos);
            // Allow Time for Arm to Move to Mat
            SystemClock.sleep(500);
            MoveGrip(PARAMS.gripOpenPos);
            return false;
        };
    }

    public Action RetractArm() {
        return packet -> {
            MoveGrip(PARAMS.gripOpenPos);
            MoveArm(PARAMS.armUpPos);
            // Allow Time for Arm to Move Off the Mat
            SystemClock.sleep(100);
            MoveFlip(PARAMS.flipSuplexPos);
            return false;
        };
    }

    /*
     * Driver Claw Movements
     */
    public void closeGrip(){
        MoveGrip(PARAMS.gripClosedPos);
    }

    public void openGrip(){
        MoveGrip(PARAMS.gripOpenPos);
    }

    public void suplexFlip() { MoveFlip(PARAMS.flipSuplexPos); }

    public void downFlip() { MoveFlip(PARAMS.flipDownPos); }

    public void moveLevel(int level){
        double pos = PARAMS.armDownPos + (level * 0.005);
        double flipPos = PARAMS.flipDownPos - (level * 0.005);
        arm.setPosition(pos);
        flip.setPosition(flipPos);
    }

    public void SuplexPixel () {
        // Pickup and Suplex Pixel
        arm.setPosition(PARAMS.armUpPos);
        DeferredActions.CreateDeferredAction(100, DeferredActionType.CLAW_FLIP_SUPLEX);
        // Wait for Pixel over Bin
        DeferredActions.CreateDeferredAction(700, DeferredActionType.CLAW_OPEN_GRIP);
    }


    public void PrepForPixel () {
        // Reset Claw to Down and Open
        grip.setPosition(PARAMS.gripOpenPos);
        arm.setPosition(PARAMS.armDownPos);
        // Wait for Arm to Separate from Bin
        DeferredActions.CreateDeferredAction(80, DeferredActionType.CLAW_FLIP_DOWN);
    }
 }









