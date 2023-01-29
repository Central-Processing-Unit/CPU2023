package org.firstinspires.ftc.teamcode.opmodes;

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

import org.firstinspires.ftc.teamcode.actions.SetClawAction;
import org.firstinspires.ftc.teamcode.util.OpModeHolder;
import org.firstinspires.ftc.teamcode.util.RobotConstants;
import org.firstinspires.ftc.teamcode.util.SignalSleeveDetector;

@Autonomous
public class RedEmergencyPark extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        int parkingPos = -200;

        Config config = new Config.Builder()
                .setDriveMotors("m0", "m1", "m2", "m3")
                .setOdometryWheelProperties(537.7f, 96, 0, 0)
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
            parkingPos = -600;
        else if (zone == SignalSleeveDetector.Zone.ZONE_THREE)
            parkingPos = 450;

        Pipeline pipeline = new Pipeline.Builder(manager)
                .addAction(new SetClawAction(manager, true))
                .addLinearPath(PrecisionMode.MEDIUM, new TrapezoidalMotionProfile(600, 1300), false,
                        new Position(0, 950, 0))
                .addLinearPath(PrecisionMode.LOW, new TrapezoidalMotionProfile(600, 1300), false,
                        new Position(-parkingPos, 950, 0))
                .build();

        pipeline.execute();
    }
}
