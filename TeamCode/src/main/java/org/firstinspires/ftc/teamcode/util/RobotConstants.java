package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.chsrobotics.ftccore.engine.navigation.control.PIDParams;
import com.chsrobotics.ftccore.engine.navigation.path.Tolerances;
import com.chsrobotics.ftccore.geometry.Position;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Config
public class RobotConstants {
    public static PIDParams rotation = new PIDParams(760, 0, 0);
    public static PIDParams linear = new PIDParams(5.3, 0, 0);
    public static Tolerances lowPrecision = new Tolerances(30, 0.15);
    public static Tolerances mediumPrecision = new Tolerances(20, 0.1);
    public static Tolerances highPrecision = new Tolerances(12, 0.08);
    public static String vuforiaKey = "AS0ENI3/////AAABmRrhaZtkGkSMi4tGQFf9azI3tZlg7Xv8GCAFy/EtV7oDQmsVBBNgiQNq035C7ShFgSt1Y9dtgOUrPHhlgoI/8sqhoBUnr3WRm/ex/gPsScPYlpy4mqBUZEIQxI2hndIuFrxPSc5gCMC4kyay2RWUWthzUygnp/22kgrq2u7xyKLwsUIctziWB1T3xreY6LcdSuqgPx6qMeiOmPkqLrIm+BbJovtmoVA7d/PqPoIeoo6O/CurFZVUeJq7zkPRB9OzsoF3Iyxyd3jGi1xlPes828QsbIcx1UYQIyR+q52fLVAt69FPPQ6AO8YMfgc0z+qF7pSA1Vee1LIyF+HCMh67gXj3YntVhvlnSeflrFtVB7vl";
    public static String asset = "/sdcard/FIRST/tflitemodels/bulldogsVision.tflite";
}
