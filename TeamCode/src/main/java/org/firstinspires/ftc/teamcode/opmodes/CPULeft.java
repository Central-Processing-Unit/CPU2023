package org.firstinspires.ftc.teamcode.opmodes;

import com.chsrobotics.ftccore.engine.navigation.control.PIDParams;
import com.chsrobotics.ftccore.engine.navigation.path.PrecisionMode;
import com.chsrobotics.ftccore.engine.navigation.path.TrapezoidalMotionProfile;
import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.hardware.config.Config;
import com.chsrobotics.ftccore.hardware.config.accessory.Accessory;
import com.chsrobotics.ftccore.hardware.config.accessory.AccessoryType;
import com.chsrobotics.ftccore.pipeline.Pipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.actions.ContinuousLiftAction;
import org.firstinspires.ftc.teamcode.actions.SetClawAction;
import org.firstinspires.ftc.teamcode.actions.UpdateLiftAction;
import org.firstinspires.ftc.teamcode.actions.WaitAction;
import org.firstinspires.ftc.teamcode.actions.WaitLiftAction;
import org.firstinspires.ftc.teamcode.util.OpModeHolder;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.SignalSleeveDetector;

@Autonomous
@com.acmerobotics.dashboard.config.Config
public class CPULeft extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        int parkingPos = -200;

        Config config = new Config.Builder()
                .setDriveMotors("m0", "m1", "m2", "m3")
                .setOdometryWheelProperties(537.7f, 48, 0, 0)
                .addAccessory(new Accessory(AccessoryType.MOTOR, "l0"))
                .addAccessory(new Accessory(AccessoryType.SERVO, "s0"))
                .addAccessory(new Accessory(AccessoryType.SERVO, "s1"))
                .setNavigationTolerances(RobotConstants.mediumPrecision)
                .setHighPrecisionTolerances(RobotConstants.highPrecision)
                .setLowPrecisionTolerances(RobotConstants.lowPrecision)
                .useDegrees(true)
                .addAccessory(new Accessory(AccessoryType.WEBCAM, "webcam"))
                .setMotorDirection(DcMotorSimple.Direction.FORWARD)
                .setIMU("imu")
                .setOpMode(this)
                .setPIDCoefficients(RobotConstants.linear, RobotConstants.rotation)
                .setDebugMode(true)
                .build();

        HardwareManager manager = new HardwareManager(config, hardwareMap);

        manager.accessoryMotors[0].setDirection(DcMotorSimple.Direction.REVERSE);

        manager.accessoryServos[0].setPosition(0.57);
        manager.accessoryServos[1].setPosition(0.46);

        OpModeHolder.opMode = this;

        SignalSleeveDetector.initAprilTags(manager);

        SignalSleeveDetector.Zone zone = SignalSleeveDetector.Zone.ZONE_TWO;

        while (!isStarted() && !isStopRequested()) {
            SignalSleeveDetector.detect(manager);
            telemetry.addData("Zone: ", SignalSleeveDetector.zone);
            telemetry.update();
        }

        waitForStart();

        zone = SignalSleeveDetector.zone;

        SignalSleeveDetector.camera.stopStreaming();

        if (zone == SignalSleeveDetector.Zone.ZONE_ONE)
            parkingPos = -670;
        else if (zone == SignalSleeveDetector.Zone.ZONE_THREE)
            parkingPos = 350;

        Pipeline pipeline = new Pipeline.Builder(manager)
                .addContinuousAction(new ContinuousLiftAction(manager))
                .addAction(new SetClawAction(manager, true))
                .addAction(new UpdateLiftAction(manager, 3000))
                .addLinearPath(PrecisionMode.MEDIUM, new TrapezoidalMotionProfile(800, 1600), false,
                        new Position(70, 1060, 310, 1.5))
                .addAction(new WaitLiftAction(manager))
                .addAction(new SetClawAction(manager, false))
                .addAction(new WaitAction(manager, 300))
                .addAction(new UpdateLiftAction(manager, 600, 300))
                .addLinearPath(PrecisionMode.LOW, new TrapezoidalMotionProfile(600, 1300), false,
                        new Position(-535, 920, 90, 1))
                .addAction(new SetClawAction(manager, true))
                .addAction(new WaitAction(manager, 300))
                .addAction(new UpdateLiftAction(manager, 1200))
                .addAction(new WaitLiftAction(manager))
                .addAction(new UpdateLiftAction(manager, 3100))
                .addLinearPath(PrecisionMode.LOW, new TrapezoidalMotionProfile(800, 1300), false,
                        new Position(200, 1060, 0, 1.2))
                .addAction(new SetClawAction(manager, false))
                .addAction(new WaitAction(manager, 300))
                .addAction(new UpdateLiftAction(manager, 500, 500))
                .addLinearPath(PrecisionMode.LOW, new TrapezoidalMotionProfile(600, 1300), false,
                        new Position(-600, 940, 90, 1.3))
                .addAction(new SetClawAction(manager, true))
                .addAction(new WaitAction(manager, 300))
                .addAction(new UpdateLiftAction(manager, 1200))
                .addAction(new WaitLiftAction(manager))
                .addAction(new UpdateLiftAction(manager, 3100))
                .addLinearPath(PrecisionMode.LOW, new TrapezoidalMotionProfile(800, 1000), false,
                        new Position(130, 1075, 0, 1.2))
                .addAction(new SetClawAction(manager, false))
                .addAction(new WaitAction(manager, 300))
                .addAction(new UpdateLiftAction(manager, 350, 500))
                .addLinearPath(PrecisionMode.LOW, new TrapezoidalMotionProfile(600, 1300), false,
                        new Position(-640, 945, 90, 1.3))
                .addAction(new SetClawAction(manager, true))
                .addAction(new WaitAction(manager, 300))
                .addAction(new UpdateLiftAction(manager, 1200))
                .addAction(new WaitLiftAction(manager))
                .addAction(new UpdateLiftAction(manager, 3100))
                .addLinearPath(PrecisionMode.LOW, new TrapezoidalMotionProfile(800, 1000), false,
                        new Position(100, 1075, 0, 1.4))
                .addAction(new SetClawAction(manager, false))
                .addAction(new UpdateLiftAction(manager, 20, 500))
                .addLinearPath(new TrapezoidalMotionProfile(700, 1500), false,
                        new Position(parkingPos, 920, 0))

                .addAction(new WaitLiftAction(manager))
                .build();

        pipeline.execute();

    }
}
