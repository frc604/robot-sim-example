package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.generated.TunerConstants;

public class Constants {
  public class Drivetrain {
    public static double kMaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // m/s
    public static double kMaxAngularRate = 1.5 * Math.PI; // rad/s
    public static double kDeadband = 0.1;
  }

  public class Intake {
    public class Arm {
      public static int deviceID = 20;
      public static double ratio = 50.0; // 50:1 reduction

      // Hard-stop limits
      public static double minAngle = Math.toRadians(-60);
      public static double maxAngle = Math.toRadians(90);
      public static double startingAngle = Math.toRadians(90);

      // Setpoints
      public static double stowAngle = Math.toRadians(80);
      public static double deployAngle = Math.toRadians(-30);

      // For simulation
      public static double simMOI = 0.2; // kgMetersSquared
      public static double simCGLength = 0.3; // m
    }

    public class Roller {
      public static int deviceID = 21;
      public static double ratio = 5.0; // 5:1 reduction

      // For simulation
      public static final double simMOI = 0.01; // kgMetersSquared
      public static final double angularVelocityScalar = 0.03;
    }
  }

  public class Transforms {
    public static Transform3d robotToCamera =
        new Transform3d(
            new Translation3d(0, 0, 0.5), // Centered on the robot, 0.5m up
            new Rotation3d(0, Math.toRadians(-15), 0)); // Pitched 15 deg up
  }
}
