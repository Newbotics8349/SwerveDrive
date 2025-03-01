package frc.robot.subsystems;

/*import com.pathplanner.lib.config.ModuleConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;

public class AntiPathplanner {
    
private final CANsparkMAX drivemotor;
private final CANsparkMAX turningmotor;

private final CANEncoder driveEncoder;
private final CANEncoder turningEncoder;

private final PIDController turningPidController;

private final AnalogInput absoluteEncoder;
private final boolean absoluteEncoderReversed;
private final double absoluteEncoderOffsetRad;

public AntiPathplanner(int drivemotorID, int turningmotorID, boolean drivemotorReversed, boolean turningmotorReversed, int AbsoluteEncoderID, double absoluteEncoderoffset, boolean absoluteEncoderReversed){
    this.absoluteEncoderOffsetRad = absoluteEncoderoffset;
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    absoluteEncoder = new AnalogInput(AbsoluteEncoderID);

   drivemotor = new CANsparkMAX(drivemotorID, MotorType.kBrushless);
   turningmotor = new CANsparkMAX(turningmotorID, MotorType.kBrushless);

   drivemotor.setInverted(drivemotorReversed);
   turningmotor.setInverted(turningmotorReversed);

driveEncoder = drivemotor.getEncoder();
turningEncoder = turningmotor.getEncoder();

driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2meter);
driveEncoder.setvelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2meterPerSec);
turningEncoder.setPositionConversionFactor(ModuleConstants.kturningencoderRot2Rad);
turningencoder.setvelocityConversionFactor(ModuleConstants.kturningencoderRPM2RadPerSec);

turningPidController = new PIDcontroller(ModuleConstants.kpturning, 0, 0);
turningPidController.enableContinuousInput(-Math.PI, Math.PI);

}

public double getDrivePosition(){
    return driveEncoder.getpositon();
}

public double getTurningPosition(){
    return turningEncoder.getpositon();
}

public double getDriveVelocity(){
    return driveEncoder.getvelocity();
}

public double getTurningvelocity(){
    return turningEncoder.getvelocity();
}

public double getAbsoluteEncoderRad(){
    double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
    angle *= 2.0 *Math.PI;
    angle -= absoluteEncoderOffsetRad;
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0 );
}
public void resetEncoders(){
    
}


}
/* */