package org.firstinspires.ftc.teamcode.actions;

import com.chsrobotics.ftccore.actions.Action;
import com.chsrobotics.ftccore.hardware.HardwareManager;

public class WaitClawAction extends Action {

    public WaitClawAction(HardwareManager hardware) {
        super(hardware);
    }

    @Override
    public void execute() {
        while (Math.abs(ContinuousClawAction.clawTarget - (hardware.accessoryMotors[1].getCurrentPosition() - 10)) > 3)
        {
            ContinuousClawAction.manualClaw();
        }
    }
}
