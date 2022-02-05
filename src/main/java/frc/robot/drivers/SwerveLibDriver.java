package frc.robot.drivers;

import com.swervedrivespecialties.swervelib.Mk3ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;

public class SwerveLibDriver implements SwerveDriver {

    private SwerveModule[] swerveModules;
    
    /**
     * Creates a new instance.
     */
    public SwerveLibDriver() {
        swerveModules = new SwerveModule[] {
            null, null, null, null
        };

        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");

        swerveModules[Constants.FRONT_LEFT] = Mk3SwerveModuleHelper.createNeo(
            shuffleboardTab.getLayout("front left module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0),
            Mk3SwerveModuleHelper.GearRatio.STANDARD, 
            Constants.FRONT_LEFT_DRIVE_MOTOR_CAN_ID, 
            Constants.FRONT_LEFT_TURN_MOTOR_CAN_ID, 
            Constants.FRONT_LEFT_TURN_MOTOR_CAN_ID,
            0    
        );

        swerveModules[Constants.FRONT_RIGHT] = Mk3SwerveModuleHelper.createNeo(
            shuffleboardTab.getLayout("front right module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0),
            Mk3SwerveModuleHelper.GearRatio.STANDARD, 
            Constants.FRONT_RIGHT_DRIVE_MOTOR_CAN_ID, 
            Constants.FRONT_RIGHT_TURN_MOTOR_CAN_ID, 
            Constants.FRONT_RIGHT_TURN_MOTOR_CAN_ID,
            0    
        );

        swerveModules[Constants.BACK_LEFT] = Mk3SwerveModuleHelper.createNeo(
            shuffleboardTab.getLayout("back left module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0),
            Mk3SwerveModuleHelper.GearRatio.STANDARD, 
            Constants.BACK_LEFT_DRIVE_MOTOR_CAN_ID, 
            Constants.BACK_LEFT_TURN_MOTOR_CAN_ID, 
            Constants.BACK_LEFT_TURN_MOTOR_CAN_ID,
            0    
        );

        swerveModules[Constants.BACK_RIGHT] = Mk3SwerveModuleHelper.createNeo(
            shuffleboardTab.getLayout("back right module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0),
            Mk3SwerveModuleHelper.GearRatio.STANDARD, 
            Constants.BACK_RIGHT_DRIVE_MOTOR_CAN_ID, 
            Constants.BACK_RIGHT_TURN_MOTOR_CAN_ID, 
            Constants.BACK_RIGHT_TURN_MOTOR_CAN_ID,
            0   
        );
   }

   /**
    * Makes the SwerveModules drive & pivot.
    *
    * @param swerveModuleStates Shopping cart angles and speeds.
    */
    @Override
    public void drive(SwerveModuleState[] swerveModuleStates) {
        double MAX_VOLTAGE = 12.0;
        
        for (int i = 0; i < 4; i++) {
            double speedPercent = swerveModuleStates[i].speedMetersPerSecond / Constants.ROBOT_MAXIMUM_SPEED_METERS_PER_SECOND;
            swerveModules[i].set(speedPercent * MAX_VOLTAGE, swerveModuleStates[i].angle.getRadians());
        }
    }
    

}