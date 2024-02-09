package org.firstinspires.ftc.teamcode.Helper;

import androidx.annotation.NonNull;
import android.util.Pair;
import java.util.ArrayList;
import java.util.List;
import static java.lang.System.currentTimeMillis;

public class DeferredActions {

    public enum DeferredActionType {
        CLAW_FLIP_SUPLEX("Claw Flip Suplex"),
        CLAW_FLIP_DOWN("Claw Flip Down"),
        CLAW_OPEN_GRIP("Claw Open Grip"),
        CLAW_ARM_SUPLEX("Claw Arm Raise");

        private final String description;

        DeferredActionType(String description) {
            this.description = description;
        }

        @Override
        public @NonNull String toString() {
            return description;
        }
    }


    // TODO : Replace Pair with a Private Static Class for Deferred Action Events
    private static volatile List<Pair<Long, DeferredActionType>> deferredActions =
            new ArrayList<>();


    // Add Deferred Action
    public static void CreateDeferredAction(long deltaMS, DeferredActionType event) {
        long triggerTime = currentTimeMillis() + deltaMS;
        deferredActions.add(new Pair<>(triggerTime, event));
    }

    // Check for Deferred Actions that are Ready to be Processed;
    public static List<DeferredActionType> GetReadyActions() {
        List<DeferredActionType>  readyActions = new ArrayList<>();
        List<Pair<Long, DeferredActionType>> removals = new ArrayList<>();

        // Build List of Actions Ready to Execute
        for (Pair act : deferredActions) {
            if (currentTimeMillis() >= (long) act.first) {
                // TODO: If action is ready to be triggered, process action and remove from queue
                readyActions.add((DeferredActionType) act.second);
                removals.add(act);
            }
        }

        // Remove Ready Actions from Deferred list
        for (Pair act : removals) { deferredActions.remove(act); }

        return (readyActions);
    }
}
