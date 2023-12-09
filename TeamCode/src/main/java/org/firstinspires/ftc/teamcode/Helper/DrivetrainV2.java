package org.firstinspires.ftc.teamcode.Helper;


import static java.lang.Thread.sleep;


import androidx.annotation.NonNull;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


import java.util.Date;


public class DrivetrainV2 {


    private static final float STRAFING_ADJUSTMENT = 1.08f;
    private static final float JOYSTICK_Y_INPUT_ADJUSTMENT = -1f;
    private static final double BRAKING_STOP_THRESHOLD = 0.25;
    private static final double BRAKING_GAIN = 0.15;
    private static final long BRAKING_INTERVAL = 100;
    private static final long BRAKING_MAXIMUM_TIME = (long) Math.ceil(1 / BRAKING_GAIN) * BRAKING_INTERVAL;


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


    public void setDriveVectorFromJoystick(float stickLeftX, float stickRightX, float stickY) {
        if (brakingOn) return;


        double forward = stickY * JOYSTICK_Y_INPUT_ADJUSTMENT;
        double strafe = stickLeftX * STRAFING_ADJUSTMENT;
        double rotate = stickRightX;


        setDriveVector(forward, strafe, rotate);
    }


    public boolean getBrakeStatus() {
        return brakingOn;
    }


    public void setBrakeStatus(boolean braking)  {
        brakingOn = braking;


        telemetryLastCalledTimestamp = new Date();
        ++telemetryBrakeCount;


        DcMotor.ZeroPowerBehavior newZeroBehavior = braking ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;


        if (drvMotorFrontLeft.getZeroPowerBehavior() != newZeroBehavior)
            drvMotorFrontLeft.setZeroPowerBehavior(newZeroBehavior);
        if (drvMotorBackLeft.getZeroPowerBehavior() != newZeroBehavior)
            drvMotorBackLeft.setZeroPowerBehavior(newZeroBehavior);
        if (drvMotorFrontRight.getZeroPowerBehavior() != newZeroBehavior)
            drvMotorFrontRight.setZeroPowerBehavior(newZeroBehavior);
        if (drvMotorBackRight.getZeroPowerBehavior() != newZeroBehavior)
            drvMotorBackRight.setZeroPowerBehavior(newZeroBehavior);


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
                timerExpired = (System.currentTimeMillis() >= (brakeStart + BRAKING_MAXIMUM_TIME));


                if (!allStop && !timerExpired) {
                    try {
                        sleep(BRAKING_INTERVAL);
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
            double newPower = curPower - (Math.signum(curPower) * BRAKING_GAIN);
            if (Math.abs(newPower) < BRAKING_STOP_THRESHOLD) newPower = 0;
            motor.setPower(newPower);
        }


        return stopped;
    }
}
