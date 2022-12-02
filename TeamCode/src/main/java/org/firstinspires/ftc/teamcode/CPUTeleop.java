package org.firstinspires.ftc.teamcode;

import com.chsrobotics.ftccore.actions.integratedactions.SampleIMUAction;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.hardware.config.Config;
import com.chsrobotics.ftccore.hardware.config.accessory.Accessory;
import com.chsrobotics.ftccore.hardware.config.accessory.AccessoryType;
import com.chsrobotics.ftccore.teleop.Drive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class CPUTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Config config = new Config.Builder()
                .setDriveMotors("m0", "m1", "m2", "m3")
                .addAccessory(new Accessory(AccessoryType.MOTOR, "l0"))
                .addAccessory(new Accessory(AccessoryType.MOTOR, "c0"))
                .setIMU("imu")
                .setTeleopValues(1, .6)
//                .addAccessory(new Accessory(AccessoryType.WEBCAM, "webcam"))
                .setOpMode(this)
                .useCV()
                .setIMUOffset(-Math.PI)
                .build();

        HardwareManager manager = new HardwareManager(config, hardwareMap);

        Drive drive = new Drive.Builder(manager)
                .bindActionToButton(Drive.Builder.GamepadButtons.A, new SampleIMUAction(manager), 1, true)
                .addUserLoop(new CPULoop(manager, this))
                .Build();

        waitForStart();
        drive.runDriveLoop();
    }
}
