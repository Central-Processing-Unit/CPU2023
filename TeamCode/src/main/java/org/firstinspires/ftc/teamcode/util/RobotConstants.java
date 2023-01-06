package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.chsrobotics.ftccore.engine.navigation.path.Tolerances;
import com.chsrobotics.ftccore.geometry.Position;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Config
public class RobotConstants {
    public static PIDCoefficients rotation = new PIDCoefficients(530, 0.04, 0.2);
    public static PIDCoefficients linear = new PIDCoefficients(4.5, 0.0012, 0.21);
    public static Tolerances lowPrecision = new Tolerances(20, 0.1);
    public static Tolerances mediumPrecision = new Tolerances(15, 0.1);
    public static Tolerances highPrecision = new Tolerances(12, 0.08);
}
