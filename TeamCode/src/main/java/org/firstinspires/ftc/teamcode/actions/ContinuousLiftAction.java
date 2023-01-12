package org.firstinspires.ftc.teamcode.actions;

import com.chsrobotics.ftccore.actions.ContinuousAction;
import com.chsrobotics.ftccore.engine.navigation.control.PID;
import com.chsrobotics.ftccore.engine.navigation.control.PIDParams;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class ContinuousLiftAction extends ContinuousAction {
    public static double targetLiftPos = 0;
    public static HardwareManager manager;
    private final static PID controller = new PID(new PIDParams(0.006, 0, 0));

    public ContinuousLiftAction(HardwareManager hardware) {
        super(hardware);
        manager = hardware;
    }

    public static void manualLift()
    {
        double liftPos = manager.getLiftMotor().getCurrentPosition();
        double liftPosError = targetLiftPos - liftPos;
        double output = controller.getOutput(liftPosError, 0);
        manager.getLiftMotor().setPower(output);
    }

    @Override
    public void execute() {
        double liftPos = hardware.getLiftMotor().getCurrentPosition();
        double liftPosError = targetLiftPos - liftPos;
        double output = controller.getOutput(liftPosError, 0);
        hardware.getLiftMotor().setPower(output);
        manager.opMode.telemetry.addData("Lift pos", hardware.getLiftMotor().getCurrentPosition());
        manager.opMode.telemetry.update();
    }

    @Override
    public void initialize() {
        targetLiftPos = 0;
    }
}
