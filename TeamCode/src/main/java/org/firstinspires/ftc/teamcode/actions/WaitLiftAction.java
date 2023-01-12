package org.firstinspires.ftc.teamcode.actions;

import com.chsrobotics.ftccore.actions.Action;
import com.chsrobotics.ftccore.hardware.HardwareManager;

public class WaitLiftAction extends Action {
    public WaitLiftAction(HardwareManager hardware) {
        super(hardware);
    }

    @Override
    public void execute() {
        while (ContinuousLiftAction.targetLiftPos - Math.abs(hardware.getLiftMotor().getCurrentPosition()) > 30)
        {
            ContinuousLiftAction.manualLift();
            hardware.opMode.telemetry.addData("Lift", hardware.getLiftMotor().getCurrentPosition());
            hardware.opMode.telemetry.update();
        }
    }
}
