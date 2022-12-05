package org.firstinspires.ftc.teamcode.actions;

import com.chsrobotics.ftccore.actions.Action;
import com.chsrobotics.ftccore.hardware.HardwareManager;

public class ClawAction extends Action {
    private boolean closed;

    public ClawAction(HardwareManager hardware, boolean closed) {
        super(hardware);
        this.closed = closed;
    }

    @Override
    public void execute() {
        if (closed)
            hardware.accessoryMotors[1].setPower(-0.4);
        else
        {
            long time = System.currentTimeMillis() + 350;

            hardware.accessoryMotors[1].setPower(0.1);

            while (System.currentTimeMillis() < time)
            {
                //Nothing
            }

            hardware.accessoryMotors[1].setPower(0);
        }


    }
}
