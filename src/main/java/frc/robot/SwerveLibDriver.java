package frc.robot;

import com.swervedrivespecialties.swervelib.Mk3ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public class SwerveLibDriver implements SwerveDriver {

    private Gyro gyro = null;
    private SwerveDriveKinematics kinematics = null;

    public SwerveLibDriver(Gyro gyro, SwerveDriveKinematics kinematics) {
        this.gyro = gyro;
        this.kinematics = kinematics;
        
    }
    // TODO : Add odometry for autonomous purposes if needed
    @Override
    public void drive(SwerveModuleState[] swerveModuleStates) {
        // Mk3SwerveModuleHelper.createNeo(gearRatio, driveMotorPort, steerMotorPort, steerEncoderPort, steerOffset)
        Mk3ModuleConfiguration foo = null;
        
    }
    
}
