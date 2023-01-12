package org.firstinspires.ftc.teamcode.actions;

import com.chsrobotics.ftccore.actions.Action;
import com.chsrobotics.ftccore.hardware.HardwareManager;

public class UpdateLiftAction extends Action {
    private int target;

    public UpdateLiftAction(HardwareManager hardware, int target) {
        super(hardware);
        this.target = target;
    }

    @Override
    public void execute() {
        ContinuousLiftAction.targetLiftPos = target;
    }
}
