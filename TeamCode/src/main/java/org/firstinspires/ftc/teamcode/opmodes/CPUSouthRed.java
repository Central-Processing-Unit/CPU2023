package org.firstinspires.ftc.teamcode.opmodes;

import com.chsrobotics.ftccore.engine.navigation.control.PIDParams;
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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.actions.ContinuousLiftAction;
import org.firstinspires.ftc.teamcode.actions.SetClawAction;
import org.firstinspires.ftc.teamcode.actions.UpdateLiftAction;
import org.firstinspires.ftc.teamcode.actions.WaitAction;
import org.firstinspires.ftc.teamcode.actions.WaitLiftAction;

@Autonomous
public class CPUSouthRed extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        int parkingPos = 0;

        Config config = new Config.Builder()
                .setDriveMotors("m0", "m1", "m2", "m3")
                .addAccessory(new Accessory(AccessoryType.MOTOR, "l0"))
                .addAccessory(new Accessory(AccessoryType.MOTOR, "c0"))
                .addAccessory(new Accessory(AccessoryType.SERVO, "s0"))
                .addAccessory(new Accessory(AccessoryType.SERVO, "s1"))
                .addAccessory(new Accessory(AccessoryType.WEBCAM, "webcam"))

                .setMotorDirection(DcMotorSimple.Direction.FORWARD)
                .setIMU("imu")
//                .addAccessory(new Accessory(AccessoryType.WEBCAM, "webcam"))
                .setOpMode(this)
                .setPIDCoefficients(new PIDParams(3.2, 0.001,0 ), new PIDParams(550, 0.04, 0))
                //.useCV()
                .setDebugMode(true)
                .build();

        HardwareManager manager = new HardwareManager(config, hardwareMap);

        manager.accessoryMotors[0].setDirection(DcMotorSimple.Direction.REVERSE);

        manager.accessoryServos[0].setPosition(0.55);
        manager.accessoryServos[1].setPosition(0.55);

        manager.accessoryServos[0].setPosition(0.45);
        manager.accessoryServos[1].setPosition(0.45);

//        CVUtility cv = null;
//        try {
//            cv = new CVUtility(manager, telemetry);
//        } catch (Exception e) {
//            telemetry.addLine("CVUtility failed to initialized");
//            telemetry.update();
//        }

        Pipeline pipeline = new Pipeline.Builder(manager)
                .addContinuousAction(new ContinuousLiftAction(manager))
                .addAction(new SetClawAction(manager, true))
                .addLinearPath(false,
                        new Position(0, 1846, 0),
                        new Position(0, 1640, 3 * Math.PI/2))
                .addAction(new UpdateLiftAction(manager, 4200))
                .addAction(new WaitLiftAction(manager))
                .addLinearPath(false,
                        new Position(130, 1620, 3 * Math.PI/2))
                .addAction(new WaitAction(manager, 300))
                .addAction(new SetClawAction(manager, false))
                .addLinearPath(true,
                        new Position(0, 1569, 3 * Math.PI/2))
                .addAction(new UpdateLiftAction(manager, 640))
                .addLinearPath(false,
                        new Position(0,1300, Math.PI/2),
                        new Position(-650, 1300, Math.PI/2))
                .addAction(new SetClawAction(manager, true))
                .addAction(new WaitAction(manager, 500))
                .addAction(new UpdateLiftAction(manager, 960))
                .addAction(new WaitLiftAction(manager))
                .addAction(new UpdateLiftAction(manager, 4200))
                .addLinearPath(false,
                        new Position(130, 1300, 3 * Math.PI/2),
                        new Position(130, 1620, 3 * Math.PI/2))
                .addAction(new SetClawAction(manager, false))
                .addAction(new WaitAction(manager, 300))
                .addLinearPath(true, //ANOTHER CYCLE
                        new Position(0, 1569, 3 * Math.PI/2))
                .addAction(new UpdateLiftAction(manager, 500))
                .addLinearPath(false,
                        new Position(0,1300, Math.PI/2),
                        new Position(-650, 1300, Math.PI/2))
                .addAction(new SetClawAction(manager, true))
                .addAction(new WaitAction(manager, 500))
                .addAction(new UpdateLiftAction(manager, 960))
                .addAction(new WaitLiftAction(manager))
                .addAction(new UpdateLiftAction(manager, 4200))
                .addLinearPath(false,
                        new Position(130, 1300, 3 * Math.PI/2),
                        new Position(130, 1620, 3 * Math.PI/2))
                .addAction(new SetClawAction(manager, false))
                .addAction(new WaitAction(manager, 300))
                .addLinearPath(false,
                        new Position(0, 1350, 0),
                        new Position(parkingPos, 1350, 0))
                .build();

        waitForStart();

//        int dots = 1;
//        if (cv != null && cv.initialized && cv.grabFrame() != null) {
//            dots = org.firstinspires.ftc.teamcode.auto.util.SignalSleeveDetector.detectOrientation(cv.grabFrame());
//
//            telemetry.addData("Dots: ", dots);
//            cv.stopStreaming();
//        } else {
//            telemetry.addLine("Signal sleeve detection failed");
//        }
//
//        parkingPos = dots == 1 ? -600 :
//                (dots == 2 ? 0 : 700);

        pipeline.execute();
    }
}
