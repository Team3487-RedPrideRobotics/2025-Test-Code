package main.java.frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;



public class SwerveJoystickCmd extends Command{
    private final main.java.frc.robot.subsystems.SwerveSubsystem swerveSubsytem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public SwerveJoysticCmd(SwerveSubsytem swerveSubsytem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, 
            Supplier<Double> turningSpdFunction, Supplier<Boolean> fieldOrientedFunction) {
        this.swerveSubsytem = swerveSubsytem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsytem);
    }

    public void execute(){
        //getting joystick inputes
        
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        //applying deadband...but fancier
        xSpeed = Math.abs(xSpeed) > DriveConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > DriveConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > DriveConstants.kDeadband ? turningSpeed : 0.0;

        // Makes it smoother. I guess we could remove it but idk
        //tldr slows acceleration
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed) 
        * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if(fieldOrientedFunction.get()){
            //relative to field
            ChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsytem.getRotation2d());
        } else {
            //relative to robot
            ChassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // convert chassis speeds to individual module states
        SwerveModlueState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // output each module states to wheels
        swerveSubsytem.setModuleStates(moduleStates);
    }
    @Override
    public void end(boolean interrupted){
        swerveSubsytem.stopModules();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
