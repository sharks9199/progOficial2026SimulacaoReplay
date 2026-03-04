package frc.robot.subsystems.climb;

public class ClimbConstants {
    
    public static final int planetaryMotorID = 20;
    public static final int clawMotorID = 21;

    public static final int climbCurrentLimit = 60;
    public static final int clawCurrentLimit = 40;

    public static final double climbP = 0.01;
    public static final double climbI = 0.0;
    public static final double climbD = 0.0;

    public static final double clawP = 0.01;
    public static final double clawI = 0.0;
    public static final double clawD = 0.0;


    // Posições do Braço (Claw/Pivot)
    public static final double clawHomePosition = 0.0;
    public static final double clawDeployedPosition = 45.0;

    // Posições do Guincho (Planetary/Winch)
    public static final double climbHomePosition = 0.0;
    public static final double climbMaxHeightPosition = 100.0;
}