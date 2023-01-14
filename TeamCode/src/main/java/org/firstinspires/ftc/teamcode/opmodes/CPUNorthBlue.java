package org.firstinspires.ftc.teamcode.opmodes;

import com.chsrobotics.ftccore.engine.navigation.control.PID;
import com.chsrobotics.ftccore.engine.navigation.control.PIDParams;
import com.chsrobotics.ftccore.engine.navigation.path.PrecisionMode;
import com.chsrobotics.ftccore.engine.navigation.path.Tolerances;
import com.chsrobotics.ftccore.engine.navigation.path.TrapezoidalMotionProfile;
import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.hardware.config.Config;
import com.chsrobotics.ftccore.hardware.config.accessory.Accessory;
import com.chsrobotics.ftccore.hardware.config.accessory.AccessoryType;
import com.chsrobotics.ftccore.management.constants.MiscConstants;
import com.chsrobotics.ftccore.pipeline.Pipeline;
import com.chsrobotics.ftccore.vision.CVUtility;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.actions.ContinuousLiftAction;
import org.firstinspires.ftc.teamcode.actions.SetClawAction;
import org.firstinspires.ftc.teamcode.actions.UpdateLiftAction;
import org.firstinspires.ftc.teamcode.actions.WaitAction;
import org.firstinspires.ftc.teamcode.actions.WaitLiftAction;
import org.firstinspires.ftc.teamcode.util.OpModeHolder;
import org.firstinspires.ftc.teamcode.util.RobotConstants;

@Autonomous
@com.acmerobotics.dashboard.config.Config
public class CPUNorthBlue extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        int parkingPos = 0;

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
                .useCV()
                .setDebugMode(true)
                .build();

        HardwareManager manager = new HardwareManager(config, hardwareMap);

        manager.accessoryMotors[0].setDirection(DcMotorSimple.Direction.REVERSE);

        manager.accessoryServos[0].setPosition(0.45);
        manager.accessoryServos[1].setPosition(0.45);

        OpModeHolder.opMode = this;

        CVUtility cv = null;
        try {
            cv = new CVUtility(manager, telemetry);
        } catch (Exception e) {
            telemetry.addLine("CVUtility failed to initialized");
            telemetry.update();
        }

        waitForStart();

        Pipeline pipeline = new Pipeline.Builder(manager)
                .addContinuousAction(new ContinuousLiftAction(manager))
                .addAction(new SetClawAction(manager, true))
                .addAction(new UpdateLiftAction(manager, 4220))
                .addLinearPath(PrecisionMode.LOW, new TrapezoidalMotionProfile(600, 1000), false,
                        new Position(20, 1160, 270, 2))
                .addAction(new WaitLiftAction(manager))
                .addAction(new SetClawAction(manager, false))
                .addAction(new WaitAction(manager, 300))
                .addAction(new UpdateLiftAction(manager, 830))
                .addLinearPath(PrecisionMode.LOW, new TrapezoidalMotionProfile(600, 1000), false,
                        new Position(0, 935, 90))
                .addLinearPath(PrecisionMode.LOW, new TrapezoidalMotionProfile(500, 900), false,
                        new Position(-510, 915, 90, 1.3))
                .addAction(new SetClawAction(manager, true))
                .addAction(new WaitAction(manager, 300))
                .addAction(new UpdateLiftAction(manager, 1500))
                .addAction(new WaitLiftAction(manager))
                .addAction(new UpdateLiftAction(manager, 4400))
                .addLinearPath(PrecisionMode.LOW, new TrapezoidalMotionProfile(500, 1000), false,
                        new Position(275, 1000, 0, 1))
                .addAction(new SetClawAction(manager, false))
                .addAction(new WaitAction(manager, 300))
                .addAction(new UpdateLiftAction(manager, 730))
                .changePID(new PIDParams(6.5, 0.0012, 0.21))
                .addLinearPath(PrecisionMode.LOW,
                        new Position(275, 910, 0))
                .changePID(new PIDParams(4.5, 0.0012, 0.21))
                .addLinearPath(PrecisionMode.LOW, new TrapezoidalMotionProfile(500, 1000), false,
                        new Position(-510, 915, 90, 1.3))
                .addAction(new SetClawAction(manager, true))
                .addAction(new WaitAction(manager, 300))
                .addAction(new UpdateLiftAction(manager, 1500))
                .addAction(new WaitLiftAction(manager))
                .addAction(new UpdateLiftAction(manager, 4400))
                .addLinearPath(PrecisionMode.LOW, new TrapezoidalMotionProfile(500, 1000), false,
                        new Position(275, 1000, 0, 1))
                .addAction(new SetClawAction(manager, false))
                .addAction(new UpdateLiftAction(manager, 100))
                .addLinearPath(new TrapezoidalMotionProfile(700, 1500), false,
                        new Position(parkingPos, 920, 0))
                .addAction(new WaitLiftAction(manager))
                .build();

        pipeline.execute();

    }
}
