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



public class SwerveSubsystem extends SubsystemBase 
{
    double maximumSpeed = Units.feetToMeters(7);
    SwerveDrive swerveDrive;

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
          (speeds, feedforwards) -> swerveDrive.setChassisSpeeds(speeds),
          new PPHolonomicDriveController(
          new PIDConstants(1, 0, 0),
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
            }

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

    public void driveFieldOriented(ChassisSpeeds velocity)
    {
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
