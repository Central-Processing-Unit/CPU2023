package org.firstinspires.ftc.teamcode.actions;

import com.chsrobotics.ftccore.actions.ContinuousAction;
import com.chsrobotics.ftccore.engine.navigation.control.PID;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class ContinuousLiftAction extends ContinuousAction {
    public static double targetLiftPos = 0;
    public static HardwareManager manager;
    private final static PID controller = new PID(new PIDCoefficients(0.005, 0, 0));
    private static double prevLiftPos;
    private static long prevTime;


    public ContinuousLiftAction(HardwareManager hardware) {
        super(hardware);
        manager = hardware;
    }

    public static void manualLift()
    {
        double liftPos = manager.getLiftMotor().getCurrentPosition();
        double liftPosError = targetLiftPos - liftPos;
        double dLiftPosError = (liftPos - prevLiftPos) / (System.currentTimeMillis() - prevTime);
        double output = controller.getOutput(liftPosError, dLiftPosError);
        manager.getLiftMotor().setPower(output);
        prevLiftPos = liftPos;
        prevTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        double liftPos = hardware.getLiftMotor().getCurrentPosition();
        double liftPosError = targetLiftPos - liftPos;
        double dLiftPosError = (liftPos - prevLiftPos) / (System.currentTimeMillis() - prevTime);
        double output = controller.getOutput(liftPosError, dLiftPosError);
        hardware.getLiftMotor().setPower(output);
        prevLiftPos = liftPos;
        prevTime = System.currentTimeMillis();
    }

    @Override
    public void initialize() {
        prevLiftPos = hardware.getLiftMotor().getCurrentPosition();
        prevTime = System.currentTimeMillis();
        targetLiftPos = 0;
    }
}
