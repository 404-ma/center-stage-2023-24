package org.firstinspires.ftc.teamcode.Helper;

import androidx.annotation.NonNull;
import android.util.Pair;
import java.util.ArrayList;
import java.util.List;
import static java.lang.System.currentTimeMillis;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class DeferredActions {

    public enum DeferredActionType {
        CLAW_FLIP_SUPLEX("Claw Flip Suplex"),
        CLAW_OPEN_GRIP("Claw Open Grip");

        private final String description;

        DeferredActionType(String description) {
            this.description = description;
        }

        @Override
        public @NonNull String toString() {
            return description;
        }
    }

    private List<Pair<Long, DeferredActionType>> deferredActions =
            new ArrayList<Pair<Long, DeferredActionType>>();

    public void DeferredActions(@NonNull HardwareMap hdwMap){
        // Initialize Objects

    }

    public void CreateDeferredAction(long deltaMS, DeferredActionType event) {
        long triggerTime = currentTimeMillis() + deltaMS;
        deferredActions.add(new Pair<>(triggerTime, event));
    }

    public void ProcessDeferredActions() {
        // TODO: Iterate over the Action List and Check if Past TriggerTime
        for (Pair act : deferredActions) {
            if (currentTimeMillis() >= (long) act.first) {
                // TODO: If action is ready to be triggered, process action and remove from queue
                switch ((DeferredActionType) act.second) {
                    case CLAW_FLIP_SUPLEX:
                        //claw.MoveFlip(ClawMoves.PARAMS.flipSuplexPos);
                        break;
                }
                deferredActions.remove(act);
            }
        }
    }
}
