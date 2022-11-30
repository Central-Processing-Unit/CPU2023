package org.firstinspires.ftc.teamcode.actions;

import com.chsrobotics.ftccore.actions.ContinuousAction;
import com.chsrobotics.ftccore.engine.navigation.control.PID;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class ContinuousLiftAction extends ContinuousAction {
    private final PID liftController = new PID(new PIDCoefficients(0.003, 0, 0));
    public static double liftTarget = 0;

    public ContinuousLiftAction(HardwareManager hardware) {
        super(hardware);
    }

    @Override
    public void execute() {
        hardware.accessoryMotors[0].setPower(liftController.getOutput(liftTarget - hardware.accessoryMotors[0].getCurrentPosition(), 0));
    }

    @Override
    public void initialize() {

    }
}
