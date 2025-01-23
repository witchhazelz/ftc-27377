package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import java.util.concurrent.Callable;

public final class Actions {
    public static class RunnableAction implements Action {
        private final Callable<Boolean> action;

        public RunnableAction(Callable<Boolean> action) {
            this.action = action;
        }

        @Override
        public boolean run(TelemetryPacket packet) {
            try {
                return action.call();
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        }
    }
}

