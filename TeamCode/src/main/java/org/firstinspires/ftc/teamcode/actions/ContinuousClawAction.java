package org.firstinspires.ftc.teamcode.actions;

import com.chsrobotics.ftccore.actions.ContinuousAction;
import com.chsrobotics.ftccore.engine.navigation.control.PID;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class ContinuousClawAction extends ContinuousAction {
    public ContinuousClawAction(HardwareManager hardware) {

        super(hardware);
        manager = hardware;
    }
    private static final PID clawController = new PID(new PIDCoefficients(0.0055, 0, 0));
    public static double clawTarget = 56;
    public static HardwareManager manager;

    @Override
    public void execute() {
        manager.accessoryMotors[1].setPower(clawController.getOutput(clawTarget - (manager.accessoryMotors[1].getCurrentPosition() - 10), 0));
    }

    public static void manualClaw()
    {
        manager.accessoryMotors[1].setPower(clawController.getOutput(clawTarget - (manager.accessoryMotors[1].getCurrentPosition() - 10), 0));
    }

    @Override
    public void initialize() {

    }
}
