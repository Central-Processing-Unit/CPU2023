package org.firstinspires.ftc.teamcode.actions;

import com.chsrobotics.ftccore.actions.Action;
import com.chsrobotics.ftccore.hardware.HardwareManager;

public class WaitLiftAction extends Action {
    public WaitLiftAction(HardwareManager hardware) {
        super(hardware);
    }

    @Override
    public void execute() {
        while (Math.abs(ContinuousLiftAction.targetLiftPos - hardware.getLiftMotor().getCurrentPosition()) < 10)
        {
            ContinuousLiftAction.manualLift();
        }
    }
}
