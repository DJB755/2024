// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import javax.lang.model.element.Parameterizable;

//import java.util.List;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
//import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
//import com.pathplanner.lib.path.PathPlannerPath;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

  private double MaxSpeed = 5 *(.75); // 6 meters per second desired top speed change the decimal for speeding up
  private double MaxAngularRate = 2 * Math.PI *(.9); // 3/4 of a rotation per second max angular velocity

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

  SendableChooser<Command> m_chooser = new SendableChooser<>();


  private final Command TwoPiece =  new SequentialCommandGroup(
           new ParallelCommandGroup(
              new PathPlannerAuto("Blue Shoot First"),
              new InstantCommand(()-> shooterSubsystem.shootFlywheel(.29))),
           new WaitCommand(1.5),
           new InstantCommand(()-> shooterSubsystem.spinBump(.4)),
           new WaitCommand(.5),
           new InstantCommand(()-> shooterSubsystem.stopFlywheel()),
           new InstantCommand(()-> shooterSubsystem.stopBump()),
           new InstantCommand(()-> intakeSubsystem.roll(1)),
           new InstantCommand(()-> shooterSubsystem.spinBump(.2)),
           new PathPlannerAuto("Speaker to Top Close Blue"),
           new WaitCommand(1.5),
           new InstantCommand(()-> shooterSubsystem.stopBump()),
           new InstantCommand(()-> shooterSubsystem.spinBump(-.2)),
           new WaitCommand(.05),
           new ParallelCommandGroup(
              new InstantCommand(()-> shooterSubsystem.shootFlywheel(.3)),
              new PathPlannerAuto("Shoot 2 Blue")),
          new WaitCommand(1.5),
          new InstantCommand(()-> shooterSubsystem.spinBump(.4)),
          new WaitCommand(2),
          new InstantCommand(()-> shooterSubsystem.stopFlywheel()),
          new InstantCommand(()-> shooterSubsystem.stopBump()),
          new InstantCommand(()-> intakeSubsystem.rollStop()));


           private final Command ShootOutside =  new SequentialCommandGroup(
           new ParallelCommandGroup(
              new PathPlannerAuto("Blue Shoot First"),
              new InstantCommand(()-> shooterSubsystem.shootFlywheel(.29))),
           new WaitCommand(1.5),
           new InstantCommand(()-> shooterSubsystem.spinBump(.4)),
           new WaitCommand(.5),
           new InstantCommand(()-> shooterSubsystem.stopFlywheel()),
           new InstantCommand(()-> shooterSubsystem.stopBump()),
           new InstantCommand(()-> intakeSubsystem.roll(1)),
           new InstantCommand(()-> shooterSubsystem.spinBump(.2)),
           new PathPlannerAuto("Speaker to Top Close Blue"),
           new WaitCommand(2),
           new InstantCommand(()-> shooterSubsystem.stopBump()),
           new InstantCommand(()-> shooterSubsystem.spinBump(-.2)),
           new WaitCommand(.05),
           new ParallelCommandGroup(
              new InstantCommand(()-> shooterSubsystem.shootFlywheel(.3)),
              new PathPlannerAuto("Shoot 2 Blue")),
          new WaitCommand(1.5),
          new InstantCommand(()-> shooterSubsystem.spinBump(.4)),
          new WaitCommand(2),
          new InstantCommand(()-> shooterSubsystem.stopFlywheel()),
          new InstantCommand(()-> shooterSubsystem.stopBump()),
          new InstantCommand(()-> intakeSubsystem.rollStop()));

           
           

  private void configureBindings() {

    //Driver Xbox Controller
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driveStick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driveStick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driveStick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    driveStick.leftTrigger().whileTrue( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driveStick.getLeftY() * MaxSpeed*(.4)) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driveStick.getLeftX() * MaxSpeed*(.4)) // Drive left with negative X (left)
            .withRotationalRate(-driveStick.getRightX() * MaxAngularRate*(.4) // Drive counterclockwise with negative X (left)
        )));    

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
    new StartEndCommand(() -> shooterSubsystem.shootFlywheel(.33), shooterSubsystem::stopFlywheel));

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

    opStick.y().whileTrue(
       new StartEndCommand(() -> shooterSubsystem.spinBump(-.5), shooterSubsystem::stopBump));

    opStick.y().whileTrue(
       new StartEndCommand(() -> intakeSubsystem.roll(-.5), intakeSubsystem::rollStop));

    
     
  //  opStick.rightTrigger().whileTrue( 
  //    new StartEndCommand(() -> climberSubsystem.climbUp(.2), climberSubsystem::stopClimb));
  //   opStick.leftTrigger().whileTrue( 
  //    new StartEndCommand(() -> climberSubsystem.climbDown(.2), climberSubsystem::stopClimb));

  }

  public RobotContainer() {
    configureBindings();
    
     m_chooser.setDefaultOption("Two Piece", TwoPiece);
     m_chooser.addOption("Shoot Outside", ShootOutside);


  }

    public Command getAutonomousCommand() {
   
      return new SequentialCommandGroup(
           new ParallelCommandGroup(
              new PathPlannerAuto("Blue Shoot First"),
              new InstantCommand(()-> shooterSubsystem.shootFlywheel(.29))),
           new WaitCommand(1.5),
           new InstantCommand(()-> shooterSubsystem.spinBump(.4)),
           new WaitCommand(.5),
           new InstantCommand(()-> shooterSubsystem.stopFlywheel()),
           new InstantCommand(()-> shooterSubsystem.stopBump()),
           new InstantCommand(()-> intakeSubsystem.roll(1)),
           new InstantCommand(()-> shooterSubsystem.spinBump(.2)),
           new PathPlannerAuto("Speaker to Top Close Blue"),
           new InstantCommand(()-> shooterSubsystem.stopBump()),
           new InstantCommand(()-> shooterSubsystem.spinBump(-.2)),
           new WaitCommand(.05),
           new ParallelCommandGroup(
              new InstantCommand(()-> shooterSubsystem.shootFlywheel(.3)),
              new PathPlannerAuto("Shoot 2 Blue")),
          new WaitCommand(1.2),
          new InstantCommand(()-> shooterSubsystem.spinBump(.4)),
          new WaitCommand(1),
          new InstantCommand(()-> shooterSubsystem.stopFlywheel()),
          new InstantCommand(()-> shooterSubsystem.stopBump()),
          new InstantCommand(()-> intakeSubsystem.rollStop())
          );

      

     // return new PathPlannerAuto("Top Blue");
}
}