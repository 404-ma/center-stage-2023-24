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

    private static class DeferredActionEvent {
        public long triggerTime;
        public DeferredActionType action;

        public DeferredActionEvent(long triggerTime, DeferredActionType action) {
            this.triggerTime = triggerTime;
            this.action = action;
        }
    }

    private static volatile List<DeferredActionEvent> deferredActions = new ArrayList<>();


    // Add Deferred Action
    public static void CreateDeferredAction(long deltaMS, DeferredActionType event) {
        long triggerTime = currentTimeMillis() + deltaMS;
        deferredActions.add(new DeferredActionEvent(triggerTime, event));
    }

    // Check for Deferred Actions that are Ready to be Processed;
    public static List<DeferredActionType> GetReadyActions() {
        List<DeferredActionType> readyActions = new ArrayList<>();
        List<DeferredActionEvent> removals = new ArrayList<>();

        // Build List of Actions Ready to Execute
        for (DeferredActionEvent act : deferredActions) {
            if (currentTimeMillis() >= (long) act.triggerTime) {
                // TODO: If action is ready to be triggered, process action and remove from queue
                readyActions.add((DeferredActionType) act.action);
                removals.add(act);
            }
        }

        // Remove Ready Actions from Deferred list
        for (DeferredActionEvent act : removals) { deferredActions.remove(act); }

        return (readyActions);
    }
}
