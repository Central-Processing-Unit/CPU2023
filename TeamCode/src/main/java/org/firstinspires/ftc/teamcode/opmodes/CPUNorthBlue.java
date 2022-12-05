package org.firstinspires.ftc.teamcode.opmodes;

import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.hardware.config.Config;
import com.chsrobotics.ftccore.hardware.config.accessory.Accessory;
import com.chsrobotics.ftccore.hardware.config.accessory.AccessoryType;
import com.chsrobotics.ftccore.pipeline.Pipeline;
import com.chsrobotics.ftccore.utilities.ComputerVision;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.actions.ClawAction;
import org.firstinspires.ftc.teamcode.actions.ContinuousLiftAction;
import org.firstinspires.ftc.teamcode.actions.UpdateLiftAction;
import org.firstinspires.ftc.teamcode.actions.WaitLiftAction;

@Autonomous
public class CPUNorthBlue extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Config config = new Config.Builder()
                .setDriveMotors("m0", "m1", "m2", "m3")
                .addAccessory(new Accessory(AccessoryType.MOTOR, "l0"))
                .addAccessory(new Accessory(AccessoryType.MOTOR, "c0"))
                .setIMU("imu")
//                .addAccessory(new Accessory(AccessoryType.WEBCAM, "webcam"))
                .setOpMode(this)
                .setPIDCoefficients(new PIDCoefficients(3.3, 0.001,0 ), new PIDCoefficients(300, 0.01, 0))
                //.useCV()
                .setDebugMode(true)
                .build();

        HardwareManager manager = new HardwareManager(config, hardwareMap);

        manager.accessoryMotors[0].setDirection(DcMotorSimple.Direction.REVERSE);

        Pipeline pipeline = new Pipeline.Builder(manager)
                .build();

        waitForStart();

        pipeline.execute();
    }
}
