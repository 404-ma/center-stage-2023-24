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
        public double armUpPos = 0.312;
        public double armDownPos = 0.237;
        public double armPickPos = 0.275; // Lift Pixel From Stack
        public double armLevel1 = 0.244;  // Pixels 2 & 3
        public double armLevel2 = 0.247;  // Pixels 3 & 4
        public double armLevel3 = 0.250;  // Pixels 4 & 5
        public double armLevel4 = 0.254;  // Top Pixel  5 Only

        public double flipSuplexPos = 0.332;
        public double flipDownPos = 0.454;

        public double gripOpenPos = 0.637;
        public double gripClosedPos = 0.355;
        public double gripOpenPosTop = 0.435;
    }

    public static Params PARAMS = new Params();

    public double tlmArmPosition = -1;
    public double tlmGripPosition = -1;
    public double tlmFlipPosition = -1;

    private final Servo arm;
    private final Servo flip;
    private final Servo grip;


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


    public Action PlacePixelAction() {
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

    public Action RetractArmAction() {
        return packet -> {
            MoveGrip(PARAMS.gripOpenPos);
            MoveArm(PARAMS.armUpPos);
            // Allow Time for Arm to Move Off the Mat
            SystemClock.sleep(100);
            MoveGrip(PARAMS.gripOpenPosTop);
            MoveFlip(PARAMS.flipSuplexPos);
            return false;
        };
    }

    public Action TopOfStackPickupAction(int level) {
        return packet -> {
            // Prep for Pickup off Stack
            PrepForPixel(false);
            SystemClock.sleep(550);
            // Allow Time for Arm to Move
            moveLevel(level);
            SystemClock.sleep(350);
            // Close if needed
            if (tlmGripPosition != PARAMS.gripClosedPos) {
                MoveGrip(PARAMS.gripClosedPos);
                SystemClock.sleep(150);
            }
            // Lift to Safe Position, Suplex later while driving
            MoveArm(PARAMS.armPickPos);
            return false;
        };
    }

    public Action SuplexPixelAction() {
        return packet -> {
            // Pickup and Suplex Pixel
            if (tlmGripPosition != PARAMS.gripClosedPos) {
                MoveGrip(PARAMS.gripClosedPos);
                SystemClock.sleep(150);
            }
            MoveArm(PARAMS.armUpPos);
            SystemClock.sleep(150);
            MoveFlip(PARAMS.flipSuplexPos);
            // Wait for Pixel over Bin
            SystemClock.sleep(675);
            MoveGrip(PARAMS.gripOpenPosTop);
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
        if (tlmArmPosition == PARAMS.armUpPos)
            MoveGrip(PARAMS.gripOpenPosTop);
        else
            MoveGrip(PARAMS.gripOpenPos);
    }

    // TODO:  Test Height Levels

    public void moveLevel(int level) {
        double armPos;
        double flipPos;
        flipPos = PARAMS.flipDownPos - 0.003;
        if (level == 1)
            armPos = PARAMS.armLevel1;
        else if (level == 2)
            armPos = PARAMS.armLevel2;
        else if (level == 3)
            armPos = PARAMS.armLevel3;
        else
            armPos = PARAMS.armLevel4;
        MoveArm(armPos);
        MoveFlip(flipPos);
    }


    public void SuplexPixel () {
        // Pickup and Suplex Pixel
        if (tlmGripPosition != PARAMS.gripClosedPos) {
            MoveGrip(PARAMS.gripClosedPos);
            DeferredActions.CreateDeferredAction(150, DeferredActionType.CLAW_ARM_UP);
        } else {
            MoveArm(PARAMS.armUpPos);
        }
        DeferredActions.CreateDeferredAction(180, DeferredActionType.CLAW_FLIP_SUPLEX);
        // Wait for Pixel over Bin
        DeferredActions.CreateDeferredAction(675, DeferredActionType.CLAW_OPEN_GRIP_UP);
    }


    public void PrepForPixel (boolean useDeferredActions) {
        // Reset Claw to Down and Open
        MoveFlip(PARAMS.flipDownPos);
        // Wait for Arm to Separate from Bin
        if (useDeferredActions) {
            DeferredActions.CreateDeferredAction(50, DeferredActionType.CLAW_ARM_DOWN);
            DeferredActions.CreateDeferredAction(100, DeferredActionType.CLAW_OPEN_GRIP_DOWN);
        } else {
            SystemClock.sleep(50);
            MoveArm(PARAMS.armDownPos);
            // Allow Time for Arm to Move to Mat
            SystemClock.sleep(100);
            MoveGrip(PARAMS.gripOpenPos);
        }
    }
 }









