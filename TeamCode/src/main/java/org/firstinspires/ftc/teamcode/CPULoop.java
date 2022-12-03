package org.firstinspires.ftc.teamcode;

import com.chsrobotics.ftccore.engine.navigation.control.PID;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.teleop.UserDriveLoop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class CPULoop extends UserDriveLoop {

    private final PID liftController = new PID(new PIDCoefficients(0.003, 0, 0));
    private double liftTarget;
    private long bLastPressed = -1;
    private boolean isClawClosed = false;


    public CPULoop(HardwareManager manager, OpMode mode) {
        super(manager, mode);
    }

    @Override
    public void loop() {
        HardwareManager hardware = this.hardware;
        DcMotorEx lift = hardware.accessoryMotors[0];
        DcMotorEx claw = hardware.accessoryMotors[1];
        Gamepad gp1 = hardware.opMode.gamepad1;

        double liftPower;
        if (gp1.right_trigger > 0.5 && (Math.abs(lift.getCurrentPosition()) < 12300 || gp1.dpad_left)) {
            liftPower = -1;
            liftTarget = lift.getCurrentPosition();
        } else if (gp1.left_trigger > 0.5 && (Math.abs(lift.getCurrentPosition()) > 0 || gp1.dpad_left)){
            liftPower = 1;
            liftTarget = lift.getCurrentPosition();
        } else {
            liftPower = liftController.getOutput(liftTarget - lift.getCurrentPosition(), 0);
            // todo: lift motor goes slower in PID. solution: override PID when it's mildly far from it's target
        }

        opmode.telemetry.addData("Loop Running", "True");
        opmode.telemetry.addData("Claw Closed", isClawClosed? "True" : "False");
        opmode.telemetry.addData("Claw Power", claw.getPower());
        opmode.telemetry.update();


        lift.setPower(liftPower);

        if (isClawClosed){
            claw.setPower(-0.3);
        }

        if (gp1.b && System.currentTimeMillis() - bLastPressed > 250) {
            bLastPressed = System.currentTimeMillis();
            isClawClosed = !isClawClosed;

            if (!isClawClosed)
            {
                double time = System.currentTimeMillis() + 300;
                double currentTime = System.currentTimeMillis();
                while (currentTime < time)
                {
                    claw.setPower(0.4);
                    currentTime = System.currentTimeMillis();
                }

                claw.setPower(0);
            }
        }
    }
}
