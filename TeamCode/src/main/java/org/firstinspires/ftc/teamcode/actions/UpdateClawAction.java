package org.firstinspires.ftc.teamcode.actions;

import com.chsrobotics.ftccore.actions.Action;
import com.chsrobotics.ftccore.hardware.HardwareManager;

public class UpdateClawAction extends Action {
    private boolean closed;

    public UpdateClawAction(HardwareManager hardware, boolean closed) {
        super(hardware);
        this.closed = closed;
    }

    @Override
    public void execute() {
        if (closed)
            ContinuousClawAction.clawTarget = 56;
        else
            ContinuousClawAction.clawTarget = 2;
    }
}
