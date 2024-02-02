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

    private ClawMoves claw;
    public DeferredActions(@NonNull HardwareMap hdwMap){
        // Initialize Objects
        claw = new ClawMoves(hdwMap);
    }

    public void CreateDeferredAction(long deltaMS, DeferredActionType event) {
        long triggerTime = currentTimeMillis() + deltaMS;
        deferredActions.add(new Pair<>(triggerTime, event));
    }

    public void ProcessDeferredActions() {
        for (Pair act : deferredActions) {
            if (currentTimeMillis() >= (long) act.first) {
                switch ((DeferredActionType) act.second) {
                    case CLAW_FLIP_SUPLEX:
                        claw.MoveFlip(ClawMoves.PARAMS.flipSuplexPos);
                        deferredActions.remove(act);
                        break;

                    case CLAW_OPEN_GRIP:
                        claw.MoveGrip(ClawMoves.PARAMS.gripOpenPos);
                        deferredActions.remove(act);
                        break;
                }

            }
        }
    }
}
