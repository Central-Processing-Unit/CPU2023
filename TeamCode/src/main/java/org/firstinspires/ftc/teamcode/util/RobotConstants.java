package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.chsrobotics.ftccore.engine.navigation.path.Tolerances;
import com.chsrobotics.ftccore.geometry.Position;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Config
public class RobotConstants {
    public static PIDCoefficients rotation = new PIDCoefficients(530, 0.04, 0.2);
    public static PIDCoefficients linear = new PIDCoefficients(4.5, 0.0012, 0.21);
    public static Tolerances lowPrecision = new Tolerances(30, 0.15);
    public static Tolerances mediumPrecision = new Tolerances(20, 0.1);
    public static Tolerances highPrecision = new Tolerances(12, 0.08);
    public static String vuforiaKey = "AS0ENI3/////AAABmRrhaZtkGkSMi4tGQFf9azI3tZlg7Xv8GCAFy/EtV7oDQmsVBBNgiQNq035C7ShFgSt1Y9dtgOUrPHhlgoI/8sqhoBUnr3WRm/ex/gPsScPYlpy4mqBUZEIQxI2hndIuFrxPSc5gCMC4kyay2RWUWthzUygnp/22kgrq2u7xyKLwsUIctziWB1T3xreY6LcdSuqgPx6qMeiOmPkqLrIm+BbJovtmoVA7d/PqPoIeoo6O/CurFZVUeJq7zkPRB9OzsoF3Iyxyd3jGi1xlPes828QsbIcx1UYQIyR+q52fLVAt69FPPQ6AO8YMfgc0z+qF7pSA1Vee1LIyF+HCMh67gXj3YntVhvlnSeflrFtVB7vl";
    public static String asset = "/sdcard/FIRST/tflitemodels/bulldogsVision.tflite";
}
