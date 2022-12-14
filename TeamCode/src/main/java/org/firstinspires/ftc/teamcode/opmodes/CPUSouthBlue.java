package org.firstinspires.ftc.teamcode.opmodes;

import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.hardware.config.Config;
import com.chsrobotics.ftccore.hardware.config.accessory.Accessory;
import com.chsrobotics.ftccore.hardware.config.accessory.AccessoryType;
import com.chsrobotics.ftccore.management.constants.MiscConstants;
import com.chsrobotics.ftccore.pipeline.Pipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import org.firstinspires.ftc.teamcode.actions.ContinuousLiftAction;
import org.firstinspires.ftc.teamcode.actions.SetClawAction;
import org.firstinspires.ftc.teamcode.actions.UpdateLiftAction;
import org.firstinspires.ftc.teamcode.actions.WaitLiftAction;

@Autonomous
public class CPUSouthBlue extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Config config = new Config.Builder()
                .setDriveMotors("m0", "m1", "m2", "m3")
                .addAccessory(new Accessory(AccessoryType.MOTOR, "l0"))
                .addAccessory(new Accessory(AccessoryType.MOTOR, "c0"))
                .setMotorDirection(DcMotorSimple.Direction.FORWARD)
                .setIMU("imu")
//                .addAccessory(new Accessory(AccessoryType.WEBCAM, "webcam"))
                .setOpMode(this)
                .setPIDCoefficients(new PIDCoefficients(3.5, 0.001,0 ), new PIDCoefficients(600, 0.04, 0))
                //.useCV()
                .setDebugMode(true)
                .build();

        HardwareManager manager = new HardwareManager(config, hardwareMap);

        manager.accessoryMotors[1].setDirection(DcMotorSimple.Direction.REVERSE);

        long time = System.currentTimeMillis() + 1000;

        while (System.currentTimeMillis() < time)
            manager.accessoryMotors[1].setPower(-0.2);

        manager.accessoryMotors[0].setDirection(DcMotorSimple.Direction.REVERSE);

        Pipeline pipeline = new Pipeline.Builder(manager)
                .addContinuousAction(new ContinuousLiftAction(manager))
                .addAction(new SetClawAction(manager, true))
                .addLinearPath(false,
                        new Position(0, 1346, Math.PI/2),
                        new Position(0, 1169, Math.PI/2))
                .addAction(new UpdateLiftAction(manager, 3000))
                .addAction(new WaitLiftAction(manager))
                .addLinearPath(false,
                        new Position(-100, 1169, Math.PI/2))
                .addAction(new SetClawAction(manager, false))
                .addAction(new SetClawAction(manager, true))
                .addAction(new UpdateLiftAction(manager, 50))
                .addAction(new WaitLiftAction(manager))
                .addLinearPath(false,
                        new Position(0, 1769, (3 * Math.PI) / 2),
                        new Position(800, 1769, (3 * Math.PI) / 2))
                .addAction(new SetClawAction(manager, true))
                .addAction(new UpdateLiftAction(manager, 3000))
                .addLinearPath(false,
                        new Position(0, 1769, Math.PI / 4))
                .addAction(new WaitLiftAction(manager))
                .addAction(new SetClawAction(manager, false))
                .addAction(new SetClawAction(manager, true))
                .build();

        MiscConstants.DISTANCE_PER_TICK *= 0.9;

        waitForStart();

        manager.accessoryMotors[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        manager.accessoryMotors[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pipeline.execute();
    }
}
