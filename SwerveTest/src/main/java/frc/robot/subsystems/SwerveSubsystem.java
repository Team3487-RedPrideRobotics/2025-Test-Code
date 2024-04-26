package main.java.frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
        private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeft_Drive_MotorPort,
            DriveConstants.kFrontLeft_Turn_MotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

        private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRight_Drive_MotorPort,
            DriveConstants.kFrontRight_Turn_MotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

        private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeft_Drive_MotorPort,
            DriveConstants.kBackLeft_Turn_MotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

        private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRight_Drive_MotorPort,
            DriveConstants.kBackRight_Turn_MotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

        private final AHRS gyro = new AHRS(SPI.Port.kMXP);

        public SwerveSubsystem(){
                //this is alot of fancy words lol
                //this means it will wait 1 second to zero the gyro
                //cause its recalibrating before that
                //the thread nonsense is allowing this and other things to run at the same time
                //so it wont slow down
                new Thread(() -> {
                        try{
                                Thread.sleep(1000);
                                zeroHeading();
                        } catch (Exception e){ 
                                 System.Out.println("Error zero'ing the NavX");                  
                        }
                }).start();
                new Thread(command).start();

        }

        public void zeroHeading(){
                gyro.reset();
        }

        public double getHeading(){
                //this math makes it so the numbers are within 0 and 360
                //normally it will loop and could go up to 720 and so on
                return Math.IEEEremainder(gyro.getAngle(), 360);
        }

        public Rotation2d getRotation2d(){
                return Rotation2d.fromDegrees(getHeading());
        }

        @Override
        public void periodic() {
                SmartDashboard.putNumber("Robot Heading", getHeading());
        }  
        
        public void stopModules(){
                frontLeft.stop();
                frontRight.stop();
                backLeft.stop();
                backRight.stop();
        }

        public void setModuleStates(SwerveModuleState[] desiredStates){
                SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
                frontLeft.setDesiredState(desiredStates[0]);
                frontRight.setDesiredState(desiredStates[1]);
                backLeft.setDesiredState(desiredStates[2]);
                backRight.setDesiredState(desiredStates[3]);
                System.out.println(desiredStates[0]);
        }
}

        