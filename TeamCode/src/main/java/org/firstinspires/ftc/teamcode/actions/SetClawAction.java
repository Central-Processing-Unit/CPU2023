package org.firstinspires.ftc.teamcode.actions;

import com.chsrobotics.ftccore.actions.Action;
import com.chsrobotics.ftccore.hardware.HardwareManager;

public class SetClawAction extends Action {
    private final boolean closed;

    public SetClawAction(HardwareManager hardware, boolean closed) {
        super(hardware);
        this.closed = closed;
    }

    @Override
    public void execute() {
        if (closed){
            hardware.accessoryServos[0].setPosition(0.3);
            hardware.accessoryServos[1].setPosition(0.3);
        } else {
            hardware.accessoryServos[0].setPosition(0);
            hardware.accessoryServos[1].setPosition(0.3);
        }
    }
}
