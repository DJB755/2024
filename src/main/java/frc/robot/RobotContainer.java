// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import java.util.List;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
//import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
//import com.pathplanner.lib.path.PathPlannerPath;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.drivetrain.generated.TunerConstants;
//import frc.robot.util.controllerUtils.MultiButton;
//import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drivetrain.*;



public class RobotContainer {

    public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
//    public final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public final VisionSubsystem visionSubsystem = new VisionSubsystem();

  private double MaxSpeed = 5 *.5; // 6 meters per second desired top speed change the decimal for speeding up
  private double MaxAngularRate = 2 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driveStick = new CommandXboxController(0); //drivestick
  private final CommandXboxController opStick = new CommandXboxController(1); // My joystick

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.025).withRotationalDeadband(MaxAngularRate * 0.025) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {

    //Driver Xbox Controller
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driveStick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driveStick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driveStick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    driveStick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driveStick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driveStick.getLeftY(), -driveStick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    driveStick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    //Op Xbox Controller
    
    opStick.x().whileTrue(
    new StartEndCommand(() -> shooterSubsystem.shootFlywheel(.30), shooterSubsystem::stopFlywheel));

    opStick.leftTrigger().whileTrue(
    new StartEndCommand(() -> shooterSubsystem.shootFlywheel(.14), shooterSubsystem::stopFlywheel));

    opStick.b().whileTrue(
       new StartEndCommand(() -> shooterSubsystem.spinBump(-.2), shooterSubsystem::stopBump));
      
    opStick.a().whileTrue(
       new StartEndCommand(() -> shooterSubsystem.spinBump(.2), shooterSubsystem::stopBump));
   
    driveStick.rightTrigger().whileTrue(
      new StartEndCommand(()-> intakeSubsystem.roll(1), intakeSubsystem::rollStop));

    driveStick.rightTrigger().whileTrue(
      new StartEndCommand(() -> shooterSubsystem.spinBump(.4), shooterSubsystem::stopBump));
     
  //  opStick.rightTrigger().whileTrue( 
  //    new StartEndCommand(() -> climberSubsystem.climbUp(.2), climberSubsystem::stopClimb));
  //   opStick.leftTrigger().whileTrue( 
  //    new StartEndCommand(() -> climberSubsystem.climbDown(.2), climberSubsystem::stopClimb));

  }

  public RobotContainer() {
    configureBindings();
    


  }

    public Command getAutonomousCommand() {
        // Load the path you want to follow using its name in the GUI
        
        //PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

       // List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile("M1 to Shoot");

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return new PathPlannerAuto("M1 + Shoot");
}
}