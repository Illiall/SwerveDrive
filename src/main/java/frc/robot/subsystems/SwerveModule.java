// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;


public class SwerveModule {
    SparkMax rotation_motor;
    double rotation_speed;

    PIDController rotation_PID;

    SparkMaxConfig rotation_motor_config;
    
    SparkMax translation_motor;
    double translation_speed;

    SparkMaxConfig translation_motor_config;

    Rotation2d rotation_offset;

    Rotation2d current_rotation;

    Rotation2d desired_rotation;

    int module_number;

    SwerveModuleState current_state;

    SwerveModuleState swerve_state;

    private final RelativeEncoder translation_encoder;

    private final RelativeEncoder rotation_encoder;

    private final CANCoder rotation_can_coder;

    private AbsoluteSensorRange range;

    @SuppressWarnings("removal")
    public SwerveModule(int module_number, int rotation_motor_id, int translation_motor_id, int rotation_can_coder_id, Rotation2d rotation_offset){
        
        this.module_number = module_number;

        this.rotation_offset = rotation_offset;

        rotation_can_coder = new CANCoder(rotation_can_coder_id);

        range = AbsoluteSensorRange.Signed_PlusMinus180;

        this.rotation_can_coder.configAbsoluteSensorRange(range);

        this.translation_motor = new SparkMax(translation_motor_id, MotorType.kBrushless);
        this.translation_encoder = this.translation_motor.getEncoder();
        this.translation_encoder.setPosition(0);
        this.translation_motor_config = new SparkMaxConfig();


        this.rotation_motor = new SparkMax(rotation_motor_id, MotorType.kBrushless);
        this.rotation_encoder = this.rotation_motor.getEncoder();
        this.translation_encoder.setPosition(0);
        this.rotation_motor_config = new SparkMaxConfig();

        this.translation_motor_config
                .idleMode(IdleMode.kBrake)
                .inverted(false)
                .smartCurrentLimit(50)
                .voltageCompensation(12);

        this.translation_motor_config.encoder
                .positionConversionFactor(Constants.Drivetrain.conversion_factors.translation_position_conversion_factor)
                .velocityConversionFactor(Constants.Drivetrain.conversion_factors.translation_velocity_conversion_factor);

        this.rotation_motor_config
                .idleMode(IdleMode.kBrake)
                .inverted(false)
                .smartCurrentLimit(20)
                .voltageCompensation(12);

        this.rotation_motor_config.encoder
                .positionConversionFactor(Constants.Drivetrain.conversion_factors.rotation_position_conversion_factor);

        this.rotation_motor_config.signals
                .externalOrAltEncoderPosition(500);

        this.translation_motor.configure(translation_motor_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.rotation_motor.configure(rotation_motor_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.rotation_PID = new PIDController(Constants.Drivetrain.PID_constants.kP, Constants.Drivetrain.PID_constants.kI, Constants.Drivetrain.PID_constants.kD);
    }

    public void reset_encoder(){
        this.translation_encoder.setPosition(0);
        this.rotation_encoder.setPosition(0);
    }

    @SuppressWarnings("removal")
    public Rotation2d get_CANCoder(){
        return Rotation2d.fromDegrees(this.rotation_can_coder.getAbsolutePosition());
    }

    public SwerveModuleState get_state(){
        return new SwerveModuleState(this.translation_encoder.getVelocity(), get_CANCoder());
    }

    //Used in PathPlanner
    //Returns the SwerveModulePosition using the translation encoder's position and the CANCoder
    public SwerveModulePosition get_module_position(){
        return new SwerveModulePosition(translation_encoder.getPosition(), get_CANCoder());
    }

    public void set_desired_state(SwerveModuleState desired_state){
        current_state = this.get_state();
        current_rotation = current_state.angle.minus(rotation_offset);

        desired_state.optimize(current_rotation);
        desired_state.cosineScale(current_rotation);

        desired_rotation = desired_state.angle.plus(new Rotation2d());

        double difference = desired_rotation.getDegrees() - current_rotation.getDegrees();

        if(Math.abs(difference) < 1){
            rotation_speed = 0;
        }else{
            rotation_speed = rotation_PID.calculate(difference, 0);
        }

        translation_speed = desired_state.speedMetersPerSecond / Constants.Drivetrain.max_speed;

        rotation_motor.set(rotation_speed);

        translation_motor.set(translation_speed * 0.25);

        SmartDashboard.putNumber("difference " + module_number, difference);
        SmartDashboard.putNumber("Rotation Speed " + module_number, rotation_speed);
        SmartDashboard.putNumber("Translation speed " + module_number, translation_speed);
        SmartDashboard.putNumber("CANCoder Mod " + module_number, get_CANCoder().getDegrees());
    }

}
