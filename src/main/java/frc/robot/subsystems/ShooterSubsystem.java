package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.TalonFXConstants;

import static frc.robot.constants.ShooterConstants.MOTION_MAGIC_ACCELERATION;
//import static frc.robot.constants.ShooterConstants.SHOOTER_F;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX topFalcon;
    private final TalonFX bottomFalcon;
    private final TalonFX bumpFalcon;
    private final DigitalInput beamBreak;
    public enum ShooterModes {
        DEFAULT,
        SPEAKER,
        AMP,
        TRAP,
        BUMP
    }

    public static ShooterModes shooterModes;

    public enum ShooterStatus {
        FORWARD,
        BACKWARD,
        OFF
    }

    public static ShooterStatus shooterStatus;
    public static double setpoint = 0;
    public static boolean atSetpoint = false;
    public static boolean isShooting = false;
    public static double velocity = 0;
    MotionMagicVelocityVoltage motionMagicVelocityVoltage;



    public ShooterSubsystem() {
        topFalcon = new TalonFX(ShooterConstants.SHOOTER_TOP_MOTOR);
        bottomFalcon = new TalonFX(ShooterConstants.SHOOTER_BOTTOM_MOTOR);
        var bottomMotorConfigurator = bottomFalcon.getConfigurator();
        var topMotorConfigurator = topFalcon.getConfigurator();
        var bottomMotorConfiguration = new TalonFXConfiguration();

        bottomMotorConfiguration.MotorOutput.Inverted = ShooterConstants.SHOOTER_INVERSION;
        bottomMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        bottomMotorConfiguration.CurrentLimits = ShooterConstants.SHOOTER_CURRENT_LIMIT;
        bottomMotorConfiguration.Slot0 = ShooterConstants.SLOT_0_CONFIGS;
        topFalcon.setControl(new Follower(bottomFalcon.getDeviceID(), true));
        shooterStatus = ShooterStatus.OFF;
         shooterModes = ShooterModes.TRAP;




         bumpFalcon = new TalonFX(ShooterConstants.SHOOTER_BUMP);

         beamBreak = new DigitalInput(ShooterConstants.BEAM_BREAK);

        motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0);
//                velocity, MOTION_MAGIC_ACCELERATION, false, SHOOTER_F, 0, false, false, false);

        var motionMagicConfigs = bottomMotorConfiguration.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = 400;
        motionMagicConfigs.MotionMagicJerk = 4000;
        bottomMotorConfigurator.apply(bottomMotorConfiguration);
        topMotorConfigurator.apply(bottomMotorConfiguration);


    }

    @Override
 public void periodic() {
//
//        MotionMagicVelocityVoltage motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(
//                velocity, 0, true, ShooterConstants.SHOOTER_F, 0, false, false, false);
//
//        switch (shooterModes) {
//            case SPEAKER:
//                shooterStatus = ShooterStatus.FORWARD;
//                //   if() Pose get y is above a certain value {
//                //   Our main linear regression for RPM}
//            break;
//
//            case BUMP:
//                velocity = ShooterConstants.BUMP_FIRE_VEL;
//                shooterStatus = ShooterStatus.FORWARD;
//                rightKraken.setControl(motionMagicVelocityVoltage);
//             break;
//            case AMP:
//                velocity = ShooterConstants.AMP_VEL;
//                shooterStatus = ShooterStatus.FORWARD;
//                rightKraken.setControl(motionMagicVelocityVoltage);
//                break;
//            case TRAP:
//                velocity = ShooterConstants.TRAP_VEL;
//                shooterStatus = ShooterStatus.FORWARD;
//                rightKraken.setControl(motionMagicVelocityVoltage);
//            case DEFAULT:
//                velocity = ShooterConstants.DEFAULT_VEL;
//                rightKraken.setControl(motionMagicVelocityVoltage);
//                break;
//        }
//
//
//
   }
    public boolean getBeamBreak(){
        return beamBreak.get();
    }

    public void spinBump() {
        bumpFalcon.set(ShooterConstants.STAGE_SPEED);
    }

    public void shootFlywheel(double speed) {
        bottomFalcon.set(speed);
        topFalcon.set(speed);
        bumpFalcon.set(0.9); //
        shooterStatus = ShooterStatus.FORWARD;
    }

    public void setMode(ShooterModes mode) {
        shooterModes = mode;
    }


    public void stopFlywheel() {
        bottomFalcon.stopMotor();
        topFalcon.stopMotor();
        bumpFalcon.stopMotor();
        shooterStatus = ShooterStatus.OFF;
    }


    public void motionMagicTest(double vel){

        topFalcon.setControl(motionMagicVelocityVoltage.withVelocity(vel));
        bottomFalcon.setControl(motionMagicVelocityVoltage.withVelocity(vel));
        bumpFalcon.set(1);


    }
//    public void testMotionMagic(double vel) {
//        MotionMagicVelocityVoltage motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(
//                vel, 0, false, SHOOTER_F, 0, false, false, false);
//        rightKraken.setControl(motionMagicVelocityVoltage);
//        leftKraken.setControl(motionMagicVelocityVoltage);
//    }
    public double getBottomEncoder() {
        return bottomFalcon.getRotorVelocity().getValue();
    }

    public  double getTopEncoder() {
        return topFalcon.getRotorVelocity().getValue();
    }

    public double getAverageEncoder() {
        return (getTopEncoder() + getBottomEncoder()) / 2;
    }

    public double getFlywheelRPM() {
        return getAverageEncoder() * ShooterConstants.PULLEY_RATIO * (600.0 / TalonFXConstants.COUNTS_PER_REV);
    }

    public double flywheelMPS(double rpm) {
        return (ShooterConstants.FLYWHEEL_DIAMETER_M * Math.PI) * (rpm * 60.0);
    }

    public void setSetpoint(double setpoint) {
        ShooterSubsystem.setpoint = setpoint;
    }

    public void setVelocity(double vel){

        topFalcon.setControl(motionMagicVelocityVoltage.withVelocity(vel));
        bottomFalcon.setControl(motionMagicVelocityVoltage.withVelocity(vel));
        System.out.println((getFlywheelRPM()));

}
}