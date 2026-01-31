package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import java.io.PrintWriter;
import java.io.FileWriter;



public class SwerveSubsystem extends SubsystemBase 
{
    double maximumSpeed = Units.feetToMeters(7);
    SwerveDrive swerveDrive;
  // Telemetry entries for Shuffleboard
  private NetworkTableEntry setpointVxEntry;
  private NetworkTableEntry measVxEntry;
  private NetworkTableEntry errVxEntry;
  private NetworkTableEntry setpointVyEntry;
  private NetworkTableEntry measVyEntry;
  private NetworkTableEntry errVyEntry;
  private NetworkTableEntry setpointOmegaEntry;
  private NetworkTableEntry measOmegaEntry;
  private NetworkTableEntry errOmegaEntry;

  private java.io.File logFile;
  private PrintWriter fileLogger;

  // Last commanded chassis speeds (setpoint) to visualize against measured velocity
  private ChassisSpeeds lastSetpoint = new ChassisSpeeds(0.0, 0.0, 0.0);

    public SwerveSubsystem()
    {
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
        
        try{
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);

        } catch(IOException e)
        {
            throw new RuntimeException(e);
        }
        
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        RobotConfig config;
        try{
          config = RobotConfig.fromGUISettings();
        

        
        AutoBuilder.configure(
          this::getPose,
          this::resetOdometry,
          this::getRobotVelocity,
          (speeds, feedforwards) -> setChassisSpeeds(speeds),
          new PPHolonomicDriveController(
          new PIDConstants(5, 0, 0 ),
          new PIDConstants(5, 0, 0)),
          config,
          () -> {
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;},
            this);} catch (Exception e)
            {
              e.printStackTrace();
            }
            var pidTab = Shuffleboard.getTab("PID");
            setpointVxEntry = pidTab.add("Setpoint Vx (m/s)", 0.0).getEntry();
            measVxEntry = pidTab.add("Measured Vx (m/s)", 0.0).getEntry();
            errVxEntry = pidTab.add("Error Vx (m/s)", 0.0).getEntry();
            setpointVyEntry = pidTab.add("Setpoint Vy (m/s)", 0.0).getEntry();
            measVyEntry = pidTab.add("Measured Vy (m/s)", 0.0).getEntry();
            errVyEntry = pidTab.add("Error Vy (m/s)", 0.0).getEntry();
            setpointOmegaEntry = pidTab.add("Setpoint Omega (rad/s)", 0.0).getEntry();
            measOmegaEntry = pidTab.add("Measured Omega (rad/s)", 0.0).getEntry();
            errOmegaEntry = pidTab.add("Error Omega (rad/s)", 0.0).getEntry();

            // Initialize CSV file logger (appends to file in working directory)
            try {
              logFile = new File("C:\\Users\\aoate\\Documents\\GitHub\\frc7196_2025\\swerve_pose_log.csv");
              boolean writeHeader = !logFile.exists() || logFile.length() == 0;
              fileLogger = new PrintWriter(new FileWriter(logFile, true), true);
              if (writeHeader) {
                fileLogger.println("timestamp,poseX,poseY,rotDeg,velVx,velVy,velOmega,setpointVx,measVx,errVx,setpointVy,measVy,errVy,setpointOmega,measOmega,errOmega");
              }
            } catch (IOException e) {
              e.printStackTrace();
              fileLogger = null;
            }
          }

    @Override
    public void periodic() {
      // Publish setpoint and measured velocities for real-time graphing
      var measured = getRobotVelocity();

      // setpoint values
      if (setpointVxEntry != null) setpointVxEntry.setDouble(lastSetpoint.vxMetersPerSecond);
      if (setpointVyEntry != null) setpointVyEntry.setDouble(lastSetpoint.vyMetersPerSecond);
      if (setpointOmegaEntry != null) setpointOmegaEntry.setDouble(lastSetpoint.omegaRadiansPerSecond);

      // measured values
      if (measVxEntry != null) measVxEntry.setDouble(measured.vxMetersPerSecond);
      if (measVyEntry != null) measVyEntry.setDouble(measured.vyMetersPerSecond);
      if (measOmegaEntry != null) measOmegaEntry.setDouble(measured.omegaRadiansPerSecond);

      // errors
      if (errVxEntry != null) errVxEntry.setDouble(lastSetpoint.vxMetersPerSecond - measured.vxMetersPerSecond);
      if (errVyEntry != null) errVyEntry.setDouble(lastSetpoint.vyMetersPerSecond - measured.vyMetersPerSecond);
      if (errOmegaEntry != null) errOmegaEntry.setDouble(lastSetpoint.omegaRadiansPerSecond - measured.omegaRadiansPerSecond);

      var pose = getPose();
      if (poseXEntry != null) poseXEntry.setDouble(pose.getX());
      if (poseYEntry != null) poseYEntry.setDouble(pose.getY());
      if (poseRotEntry != null) poseRotEntry.setDouble(pose.getRotation().getDegrees());

      // optional CSV logging
      if (fileLogger != null) {
        long ts = System.currentTimeMillis();
        fileLogger.printf("%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
            ts,
            getPose().getX(),
            getPose().getY(),
            getPose().getRotation().getDegrees(),
            measured.vxMetersPerSecond,
            measured.vyMetersPerSecond,
            measured.omegaRadiansPerSecond,
            lastSetpoint.vxMetersPerSecond,
            measured.vxMetersPerSecond,
            lastSetpoint.vxMetersPerSecond - measured.vxMetersPerSecond,
            lastSetpoint.vyMetersPerSecond,
            measured.vyMetersPerSecond,
            lastSetpoint.vyMetersPerSecond - measured.vyMetersPerSecond,
            lastSetpoint.omegaRadiansPerSecond,
            measured.omegaRadiansPerSecond,
            lastSetpoint.omegaRadiansPerSecond - measured.omegaRadiansPerSecond
        );
      }

      // publish current odometry pose to Shuffleboard
      var pose = getPose();
      if (poseXEntry != null) poseXEntry.setDouble(pose.getX());
      if (poseYEntry != null) poseYEntry.setDouble(pose.getY());
      if (poseRotEntry != null) poseRotEntry.setDouble(pose.getRotation().getDegrees());
    }

    /**
     * @return The current pose of the robot.
     */

    public Pose2d getPose()
    {
      return swerveDrive.getPose();
    }
    public ChassisSpeeds getRobotVelocity()
    {
      return swerveDrive.getRobotVelocity();
    }
    public void resetOdometry(Pose2d initialHolomicPose)
    {
      swerveDrive.resetOdometry(initialHolomicPose);
    }

    public void setChassisSpeeds(ChassisSpeeds velocity)
    {
      lastSetpoint = velocity;
      swerveDrive.setChassisSpeeds(velocity);
    }

    public void driveFieldOriented(ChassisSpeeds velocity)
    {
      // store the commanded setpoint so we can visualize it against measured velocity
      lastSetpoint = velocity;
      swerveDrive.driveFieldOriented(velocity);
    }

      /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY)
  {
    return run(() -> {

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                                                                                 translationY.getAsDouble()), 1);

      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                                                                      headingX.getAsDouble(),
                                                                      headingY.getAsDouble(),
                                                                      swerveDrive.getOdometryHeading().getRadians(),
                                                                      swerveDrive.getMaximumChassisVelocity()));
    });
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction.
   * @param translationY     Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                                          translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                        angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                        true,
                        false);
    });
  }

  
  public Command alignToTag(VisionSubsystem vision)
  {

    return run(() -> {
      double tx = vision.getXYA()[0];

     // System.out.println(tx);

      ChassisSpeeds desiredSpeeds = new ChassisSpeeds(0, 0, -0.07*tx);

      this.driveFieldOriented(desiredSpeeds);
    });
  }


  public Command driveFieldOrientedCommand(Supplier<ChassisSpeeds> velocity) {
    return this.run(() -> {
      lastSetpoint = velocity.get();
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  public SwerveDrive getSwerveDrive()
  {
    return swerveDrive;
  }

  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }
    
}
