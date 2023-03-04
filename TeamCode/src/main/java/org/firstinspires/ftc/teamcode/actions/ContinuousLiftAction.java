package org.firstinspires.ftc.teamcode.actions;

import com.chsrobotics.ftccore.actions.ContinuousAction;
import com.chsrobotics.ftccore.engine.navigation.control.PID;
import com.chsrobotics.ftccore.engine.navigation.control.PIDParams;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ContinuousLiftAction extends ContinuousAction {
    public static double targetLiftPos = 0;
    public static HardwareManager manager;
    public static int time = 0;
    private static boolean waiting = false;
    private ElapsedTime timer = new ElapsedTime();
    private final static PID controller = new PID(new PIDParams(0.008, 0, 0));

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
        if (!waiting && time > 0) {
            waiting = true;
            timer.reset();
            timer.startTime();
        }

        if (waiting && timer.milliseconds() < time)
            return;
        else if (waiting && time > timer.milliseconds()) {
            waiting = false;
            time = 0;
        }

        double liftPos = hardware.getLiftMotor().getCurrentPosition();
        double liftPosError = targetLiftPos - liftPos;
        double output = controller.getOutput(liftPosError, 0);
        hardware.getLiftMotor().setPower(output);
    }

    @Override
    public void initialize() {
        targetLiftPos = 0;
    }
}
