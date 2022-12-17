package org.firstinspires.ftc.teamcode.actions;

import com.chsrobotics.ftccore.actions.Action;
import com.chsrobotics.ftccore.hardware.HardwareManager;

public class WaitAction extends Action {
    private long time;

    public WaitAction(HardwareManager hardware, int time) {
        super(hardware);
        this.time = time;
    }

    @Override
    public void execute() {
        time += System.currentTimeMillis();

        while (System.currentTimeMillis() < time) {
            //Do nothing
        }
    }
}
