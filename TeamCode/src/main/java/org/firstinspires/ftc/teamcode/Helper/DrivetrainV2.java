package org.firstinspires.ftc.teamcode.Helper;


import static java.lang.Thread.sleep;


import androidx.annotation.NonNull;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


import java.util.Date;

@Config
public class DrivetrainV2 {

    public static class Params {
        private static final float strafingAdjustment = 1.08f;
        private static final float joystickYInputAdjustment  = -1f;
        private static final double brakingStopThreshold = 0.25;
        private static final double brakingGain = 0.15;
        private static final long brakingInterval = 100;
        private static final long brakingMaximumTime = (long) Math.ceil(1 / brakingGain) * brakingInterval ;

    }

    public static DrivetrainV2.Params PARAMS = new DrivetrainV2.Params();


    private final DcMotor drvMotorFrontLeft;
    private final DcMotor drvMotorBackLeft;
    private final DcMotor drvMotorFrontRight;
    private final DcMotor drvMotorBackRight;
    private volatile boolean brakingOn = false;


    private Date telemetryLastCalledTimestamp = new Date();
    private double telemetryLastPowerFrontLeft = 0f;
    private double telemetryLastPowerBackLeft = 0f;
    private double telemetryLastPowerFrontRight = 0f;
    private double telemetryLastPowerBackRight = 0f;
    private int telemetryBrakeCount = 0;
    private int telemetryBrakeTimeoutCount = 0;

    public DrivetrainV2 (@NonNull HardwareMap hdwMap) {
        drvMotorFrontLeft = hdwMap.dcMotor.get("frontLeft");
        drvMotorBackLeft = hdwMap.dcMotor.get("backLeft");
        drvMotorFrontRight = hdwMap.dcMotor.get("frontRight");
        drvMotorBackRight = hdwMap.dcMotor.get("backRight");

        // Account for motor mounting direction in our robot design
        drvMotorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        drvMotorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        drvMotorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drvMotorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drvMotorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drvMotorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public Date getTelemetryLastCalledTimestamp() {
        return telemetryLastCalledTimestamp;
    }


    public double getTelemetryLastPowerFrontLeft() {
        return telemetryLastPowerFrontLeft;
    }


    public double getTelemetryLastPowerBackLeft() {
        return telemetryLastPowerBackLeft;
    }


    public double getTelemetryLastPowerFrontRight() {
        return telemetryLastPowerFrontRight;
    }


    public double getTelemetryLastPowerBackRight() {
        return telemetryLastPowerBackRight;
    }


    public int getTelemetryBrakeCount() {
        return telemetryBrakeCount;
    }


    public int getTelemetryBrakeTimeoutCount() {
        return telemetryBrakeTimeoutCount;
    }


    public void setDriveVector(double forward, double strafe, double rotate) {
        if (brakingOn) return;

        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        double pwrFrontLeft = (forward + strafe + rotate) / denominator;
        double pwrBackLeft = (forward - strafe + rotate) / denominator;
        double pwrFrontRight = (forward - strafe - rotate) / denominator;
        double pwrBackRight = (forward + strafe - rotate) / denominator;

        drvMotorFrontLeft.setPower(pwrFrontLeft);
        drvMotorBackLeft.setPower(pwrBackLeft);
        drvMotorFrontRight.setPower(pwrFrontRight);
        drvMotorBackRight.setPower(pwrBackRight);

        telemetryLastCalledTimestamp = new Date();
        telemetryLastPowerFrontLeft = pwrFrontLeft;
        telemetryLastPowerBackLeft = pwrBackLeft;
        telemetryLastPowerFrontRight = pwrFrontRight;
        telemetryLastPowerBackRight = pwrBackRight;
    }


    public void setDriveVectorFromJoystick(float stickLeftX, float stickRightX, float stickLeftY,boolean setReversed) {
        if (brakingOn) return;

        double rotate = stickRightX;
        double forward = stickLeftY * PARAMS.joystickYInputAdjustment;
        double strafe = stickLeftX * PARAMS.strafingAdjustment;

        if (setReversed) {
            forward = stickLeftY * PARAMS.joystickYInputAdjustment * -1;
            strafe = stickLeftX * PARAMS.strafingAdjustment * -1;
        }

        setDriveVector(forward, strafe, rotate);
    }



    public void setBrakeStatus(boolean braking)  {
        brakingOn = braking;

        telemetryLastCalledTimestamp = new Date();
        ++telemetryBrakeCount;

        if (braking) {
            boolean allStop = false;
            boolean timerExpired = false;
            long brakeStart = System.currentTimeMillis();


            while (!allStop && !timerExpired) {
                boolean flStop = coasterBrakeMotor(drvMotorFrontLeft);
                boolean blStop = coasterBrakeMotor(drvMotorBackLeft);
                boolean frStop = coasterBrakeMotor(drvMotorFrontRight);
                boolean brStop = coasterBrakeMotor(drvMotorBackRight);


                allStop = flStop && blStop && frStop && brStop;
                timerExpired = (System.currentTimeMillis() >= (brakeStart + PARAMS.brakingMaximumTime));


                if (!allStop && !timerExpired) {
                    try {
                        sleep(PARAMS.brakingInterval);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                }
            }

            if (timerExpired) ++telemetryBrakeTimeoutCount;
        }
    }


    private boolean coasterBrakeMotor(DcMotor motor) {
        double curPower = motor.getPower();
        boolean stopped = (curPower == 0);


        if (!stopped) {
            double newPower = curPower - (Math.signum(curPower) * PARAMS.brakingInterval);
            if (Math.abs(newPower) < PARAMS.brakingStopThreshold) newPower = 0;
            motor.setPower(newPower);
        }


        return stopped;
    }
}
