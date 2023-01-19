package org.firstinspires.ftc.teamcode.actions;

import com.chsrobotics.ftccore.actions.Action;
import com.chsrobotics.ftccore.hardware.HardwareManager;

public class UpdateLiftAction extends Action {
    private int target;
    private int time;

    public UpdateLiftAction(HardwareManager hardware, int target) {
        super(hardware);
        this.target = target;
    }

    public UpdateLiftAction(HardwareManager hardware, int target, int timeMilliseconds) {
        super(hardware);
        this.target = target;
        this.time = timeMilliseconds;
    }

    @Override
    public void execute() {
        ContinuousLiftAction.targetLiftPos = target;
        ContinuousLiftAction.time = time;
    }
}
