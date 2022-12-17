package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;

import com.chsrobotics.ftccore.engine.navigation.control.PID;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.teleop.UserDriveLoop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class CPULoop extends UserDriveLoop {

    private final PID liftController = new PID(new PIDCoefficients(0.003, 0, 0));private double liftTarget;
    private double liftLimit = 11200;
    private long bLastPressed = -1;
    private boolean isClawClosed = false;


    public CPULoop(HardwareManager manager, OpMode mode) {
        super(manager, mode);
    }

    public double getLiftSpeed(double input)
    {
        return (Math.sin((input * (Math.PI/2)) - (Math.PI / 2d))) + 1;
    }

    @Override
    public void loop() {
        HardwareManager hardware = this.hardware;
        DcMotorEx lift = hardware.accessoryMotors[0];
       // DcMotorEx claw = hardware.accessoryMotors[1];
        Gamepad gp1 = hardware.opMode.gamepad1;
        Gamepad gp2 = hardware.opMode.gamepad2;

        if (gp2.dpad_left)
            liftLimit = 4100;
        else if (gp2.dpad_down)
            liftLimit = 2000;
        else if (gp2.dpad_right)
            liftLimit = 1000;

        double liftPower = 0;
        if ((gp1.right_trigger > 0.01 || gp2.right_trigger > 0.01) && (Math.abs(lift.getCurrentPosition()) < liftLimit  || gp1.dpad_left)) {
            if (gp1.right_trigger > 0.01)
                liftPower = -getLiftSpeed(gp1.right_trigger);
            else if (gp2.right_trigger > 0.01)
                liftPower = -getLiftSpeed(gp2.right_trigger);
            liftTarget = lift.getCurrentPosition();
        } else if ((gp1.left_trigger > 0.01 || gp2.left_trigger > 0.01) && (lift.getCurrentPosition() < 0 || gp1.dpad_left)){
            if (gp1.left_trigger > 0.01)
                liftPower = getLiftSpeed(gp1.left_trigger);
            else if (gp2.left_trigger > 0.01)
                liftPower = getLiftSpeed(gp2.left_trigger);
            liftTarget = lift.getCurrentPosition();
        } else {
            liftPower = liftController.getOutput(liftTarget - lift.getCurrentPosition(), 0);
            // todo: lift motor goes slower in PID. solution: override PID when it's mildly far from it's target
        }

        opmode.telemetry.addData("Loop Running", "True");
        opmode.telemetry.addData("Claw Closed", isClawClosed? "True" : "False");
        //opmode.telemetry.addData("Claw Power", claw.getPower());
        opmode.telemetry.addData("Lift Power", liftPower);
        opmode.telemetry.addData("Slide", hardware.accessoryMotors[0].getCurrentPosition());
       // opmode.telemetry.addData("Claw Ticks", hardware.accessoryMotors[1].getCurrentPosition());
        opmode.telemetry.update();


        lift.setPower(liftPower);

        if (gp1.a)
        {
            hardware.IMUReset = hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, ZYX, AngleUnit.RADIANS ).firstAngle;
        }

        if ((gp1.b || gp2.b) && System.currentTimeMillis() - bLastPressed > 250) {
            bLastPressed = System.currentTimeMillis();
            isClawClosed = !isClawClosed;

            if (isClawClosed){
                hardware.accessoryServos[0].setPosition(0.495);
                hardware.accessoryServos[1].setPosition(0.495);
            } else {
                hardware.accessoryServos[0].setPosition(0.59);
                hardware.accessoryServos[1].setPosition(0.41);
            }
        }
    }
}
