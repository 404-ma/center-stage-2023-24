package org.firstinspires.ftc.teamcode.Helper;

/* Copyright (c) 2023 FIRST Tech Challenge - Team #404 “=ma” (https://...)
 *
 * Permission is granted, free of charge, to any person obtaining a copy of this software and
 * associated documentation (collectively, the "Software") to use the Software without restriction,
 * including without limitation, the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so; subject to the copyright notice above and this permission notice being
 * shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS"WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

import com.qualcomm.robotcore.hardware.Gamepad;
import static android.os.SystemClock.sleep;
import androidx.annotation.NonNull;
import java.util.Date;


/*
 * This class demonstrates how to extend an external "hardware" class (Gamepad) in order to
 * modularize the processing of the robot's sensors and actuators.
 *
 * It reads all the inputs from Gamepad class and converts them to an enumerated InputType
 * which it assigns to the the highest priority input at that moment.
 *
 * The class also handle issues related to human inputs, such as enforcing lockout intervals
 * to prevent duplicate inputs from a control being held. This is supported in three modes:
 *
 *     Button:  Restricts input of same button for a the lockout interval
 *     DPad:    Extends Button restriction but hardware only allows on
 *     Trigger: Detects edge transition ON/OFF
 *     Joystick: A very small lockout interval reduces the number of times the same
 *               joystick position registers as a new input
 *
 *  This class collects telemetry data about its most recent values and exposes
 *  that data via (getTelemetry_...) methods.
 */

public class gamePadInput {



    public enum GameplayInputType {
        NONE("No Input"),
        BUTTON_A("A Button"),
        BUTTON_B("B Button"),
        BUTTON_X("X Button"),
        BUTTON_Y("Y Button"),
        BUTTON_L_BUMPER("L Bumper"),
        BUTTON_R_BUMPER("R Bumper"),
        LEFT_TRIGGER_ON("L Trigger: ON"),
        LEFT_TRIGGER_OFF("L Trigger: OFF"),
        RIGHT_TRIGGER_ON("R Trigger: ON"),
        RIGHT_TRIGGER_OFF("R Trigger: OFF"),
        DPAD_UP("DPad: UP"),
        DPAD_DOWN("DPad: DOWN"),
        DPAD_LEFT("DPad: LEFT"),
        DPAD_RIGHT("DPad: RIGHT"),
        JOYSTICK("Joystick");

        private final String description;

        GameplayInputType(String description) {
            this.description = description;
        }

        @Override
        public @NonNull String toString() {
            return description;
        }
    }


    //--------------------------------------------------------------
    //                        CONSTANTS
    // -------------------------------------------------------------
    //  TO DO: Move these constants to a configuration file

    // Timeouts Needed to Debounce Gamepad Inputs (Milliseconds)
    static final long WAIT_LOOP_SlEEP_INTERVAL = 20;  // This Interval should be Small
    static final long BUTTON_LOCKOUT_INTERVAL = 1000;
    static final long DPAD_LOCKOUT_INTERVAL = 1000;
    static final long JOYSTICK_LOCKOUT_INTERVAL = 20;  // should be Small


    // Telemetry Data
    private int tlm_WaitLoopCount = 0;
    private Date tlm_WaitLoopLastTimestamp = new Date();
    private int tlm_InputCount = 0;
    private GameplayInputType tlm_InputLastType = GameplayInputType.NONE;
    private Date tlm_InputLastTimestamp = new Date();




    // Internal Variables - Gamepad, Lockout Counters, and Previous State Variables
    private final Gamepad inputGPad;
    private long LastButtonInputTime = 0;
    private GameplayInputType LastButtonInput = GameplayInputType.NONE;
    private boolean LeftTriggerOn = false;
    private boolean RightTriggerOn = false;
    private long LastDPadInputTime = 0;
    private GameplayInputType LastDPadInput = GameplayInputType.NONE;
    private long LastJoystickInputTime = 0;
    private float LastLeftJoystickX = 0f;
    private float LastLeftJoystickY = 0f;
    private float LastRightJoystickX = 0f;
    private float LastRightJoystickY = 0f;



    //--------------------------------------------------------------
    //                External Class Interface
    // -------------------------------------------------------------

    // Class Constructor
    public gamePadInput(@NonNull Gamepad gp) {
        this.inputGPad = gp;
    }

    // Telemetry Data Getters
    public int getTelemetry_WaitLoopCount() { return(tlm_WaitLoopCount); }
    public Date getTelemetry_WaitLoopLastTimestamp() { return(tlm_WaitLoopLastTimestamp); }
    public int getTelemetry_InputCount() { return(tlm_InputCount); }
    public GameplayInputType getTelemetry_InputLastType() { return(tlm_InputLastType); }
    public Date getTelemetry_InputLastTimestamp() { return(tlm_InputLastTimestamp); }



    /*
     * WaitForGamepadInput:  Repeatedly checks for Gamepad Inputs until Timeout expires.
     */
    public GameplayInputType WaitForGamepadInput(int msTimeout) {
        // Loop until a Gamepad input or specified timeout (milliseconds)
        long ms_start = System.currentTimeMillis();
        long ms_end = ms_start + msTimeout;

        GameplayInputType newInput = GameplayInputType.NONE;

        while (System.currentTimeMillis() < ms_end && newInput == GameplayInputType.NONE) {
            ++tlm_WaitLoopCount;
            tlm_WaitLoopLastTimestamp = new Date();

            newInput = GetGamepadInput();
            if (newInput == GameplayInputType.NONE)
                sleep(WAIT_LOOP_SlEEP_INTERVAL);
            else {
                // Return New Input
                ++tlm_InputCount;
                tlm_InputLastTimestamp = new Date();
                tlm_InputLastType = newInput;
            }
        }

        return newInput;
    }


    //--------------------------------------------------------------
    //                Internal Helper Functions
    // -------------------------------------------------------------

    /*
     * GetInput:  Checks for Gamepad Inputs in Order of Importance (most to least)
     */
    private GameplayInputType GetGamepadInput() {
        // Check for Buttons
        GameplayInputType intype = GetButton();
        if (intype != GameplayInputType.NONE) return (intype);

        // Check for Triggers
        intype = GetTrigger();
        if (intype != GameplayInputType.NONE) return (intype);

        // Check for DPad
        intype = GetDPad();
        if (intype != GameplayInputType.NONE) return (intype);

        // Check for Joystick
        intype = GetJoystick();  // Defaults to InputType None
        return (intype);
    }


    /*
     * GetButton: Check for Gamepad Button Inputs and Disregards Duplicates During Lockout Period.
     *            Buttons are checked in Order of Importance (most to least).
     *            Returns an input type of NONE when no button is pressed.
     */
    private GameplayInputType GetButton() {
        GameplayInputType intype = GameplayInputType.NONE;

        // Check for Button Inputs
        if (inputGPad.a) intype = GameplayInputType.BUTTON_A;
        if (inputGPad.b) intype = GameplayInputType.BUTTON_B;
        if (inputGPad.x) intype = GameplayInputType.BUTTON_X;
        if (inputGPad.y) intype = GameplayInputType.BUTTON_Y;
        if (inputGPad.left_bumper) intype = GameplayInputType.BUTTON_L_BUMPER;
        if (inputGPad.right_bumper) intype = GameplayInputType.BUTTON_R_BUMPER;

        // Check For Duplicate Button Input and Disregard Same Button During Lockout Period
        boolean lockedOut = ((LastButtonInputTime + BUTTON_LOCKOUT_INTERVAL) - System.currentTimeMillis()) > 0;

        if (intype == LastButtonInput && lockedOut) {
            intype = GameplayInputType.NONE;
        } else {
            LastButtonInput = intype;
            LastButtonInputTime = System.currentTimeMillis();
        }
        return (intype);
    }


    /*
     * Get Trigger: Checks for Gamepad Trigger Input Changes and Disregard Changes During the
     *              Lockout Period.  It converts the floating Trigger value to an ON/OFF Input
     *                 Trigger is considered ON when it transitions over 75%
     *                 Trigger is considered OFF when it transitions under 25%
     *              Returns an input type of NONE when no Trigger change is detected.
     */
    private GameplayInputType GetTrigger() {
        if (inputGPad.left_trigger >= 0.75f && !LeftTriggerOn) {
            LeftTriggerOn = true;
            return (GameplayInputType.LEFT_TRIGGER_ON);
        } else if (inputGPad.left_trigger < 0.25f && LeftTriggerOn) {
            LeftTriggerOn = false;
            return (GameplayInputType.LEFT_TRIGGER_OFF);
        }

        if (inputGPad.right_trigger >= 0.75f && !RightTriggerOn) {
            RightTriggerOn = true;
            return (GameplayInputType.RIGHT_TRIGGER_ON);
        } else if (inputGPad.right_trigger < 0.25f && RightTriggerOn) {
            RightTriggerOn = false;
            return (GameplayInputType.RIGHT_TRIGGER_OFF);
        }

        return (GameplayInputType.NONE); // No Trigger Changes Found
    }


    /*
     * Get DPad: Checks for Gamepad DPad (Upper Left) Input Changes and Disregards Changes During
     *           the Lockout Period.
     *           Returns an input type of NONE when no Trigger change is detected.
     */
    private GameplayInputType GetDPad() {
        GameplayInputType intype = GameplayInputType.NONE;

        // Check for DPad Inputs
        if (inputGPad.dpad_up) intype = GameplayInputType.DPAD_UP;
        if (inputGPad.dpad_down) intype = GameplayInputType.DPAD_DOWN;
        if (inputGPad.dpad_left) intype = GameplayInputType.DPAD_LEFT;
        if (inputGPad.dpad_right) intype = GameplayInputType.DPAD_RIGHT;

        // Check For Duplicate DPad Input and Disregard Same Input During Lockout Period
        boolean lockedOut = ((LastDPadInputTime + DPAD_LOCKOUT_INTERVAL) - System.currentTimeMillis()) > 0;

        if (intype == LastDPadInput && lockedOut) {
            intype = GameplayInputType.NONE;
        } else {
            LastDPadInput = intype;
            LastDPadInputTime = System.currentTimeMillis();
        }
        return (intype);
    }

    /*
     * Get Joystick: Checks for Change in Joystick Input Changes and Disregards Changes During
     *               the Lockout Period.
     *               Returns an input type of NONE when no Joystick change is detected.
     *    IMPORTANT: Joysticks are heavily used, their Lockout period should be small
     */
    private GameplayInputType GetJoystick() {
        // Check for Change in Joystick Position
        boolean newPosition = (inputGPad.left_stick_x != LastLeftJoystickX) ||
                (inputGPad.left_stick_y != LastLeftJoystickY) ||
                (inputGPad.right_stick_x != LastRightJoystickX) ||
                (inputGPad.right_stick_y != LastRightJoystickY);

        // Check for Joystick at Rest
        boolean atRest = (inputGPad.left_stick_x == 0f) && (inputGPad.left_stick_y == 0f) &&
                (inputGPad.right_stick_x == 0f) && (inputGPad.right_stick_y == 0f);

        boolean lockedOut = ((LastJoystickInputTime + JOYSTICK_LOCKOUT_INTERVAL) - System.currentTimeMillis()) > 0;

        // Joystick Moved or Remaining in Same (Non Resting) Position Past Lockout Interval
        if (!lockedOut) {
            if (newPosition || !atRest) {
                LastJoystickInputTime = System.currentTimeMillis();
                LastLeftJoystickX = inputGPad.left_stick_x;
                LastLeftJoystickY = inputGPad.left_stick_y;
                LastRightJoystickX = inputGPad.right_stick_x;
                LastRightJoystickY = inputGPad.right_stick_y;
                return (GameplayInputType.JOYSTICK);
            }
        }

        return (GameplayInputType.NONE);  // Catch all for No Joystick Input
    }


}

