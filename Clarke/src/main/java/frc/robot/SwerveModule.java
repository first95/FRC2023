package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.BetterSwerveModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants.Drivebase;

public class SwerveModule {
    public final int moduleNumber;
    private final double angleOffset;
    private final CANSparkMax angleMotor, driveMotor;
    private final AbsoluteEncoder absoluteEncoder;
    private final RelativeEncoder driveEncoder;
    private final SparkMaxPIDController angleController, driveController;
    private final Timer time;
    private double angle, lastAngle, omega, speed, fakePos, lastTime, dt;

    private SimpleMotorFeedforward feedforward;

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        angle = 0;
        speed = 0;
        fakePos = 0;
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;

        angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        angleMotor.restoreFactoryDefaults();
        driveMotor.restoreFactoryDefaults();

        // Config angle encoders
        absoluteEncoder = angleMotor.getAbsoluteEncoder(Type.kDutyCycle);
        absoluteEncoder.setPositionConversionFactor(Drivebase.DEGREES_PER_STEERING_ROTATION);
        absoluteEncoder.setVelocityConversionFactor(Drivebase.DEGREES_PER_STEERING_ROTATION / 60);
        absoluteEncoder.setZeroOffset(angleOffset);
        absoluteEncoder.setInverted(Drivebase.ABSOLUTE_ENCODER_INVERT);
        absoluteEncoder.setAverageDepth(1);

        // Config angle motor/controller
        angleController = angleMotor.getPIDController();
        angleController.setP(Drivebase.MODULE_KP);
        angleController.setI(Drivebase.MODULE_KI);
        angleController.setD(Drivebase.MODULE_KD);
        angleController.setFF(Drivebase.MODULE_KF);
        angleController.setIZone(Drivebase.MODULE_IZ);
        angleController.setPositionPIDWrappingEnabled(true);
        angleController.setPositionPIDWrappingMaxInput(180);
        angleController.setPositionPIDWrappingMinInput(-180);
        angleController.setFeedbackDevice(absoluteEncoder);
        angleMotor.setInverted(Drivebase.ANGLE_MOTOR_INVERT);
        angleMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        angleMotor.setSmartCurrentLimit(Drivebase.SWERVE_MODULE_CURRENT_LIMIT);

        // Config drive motor/controller
        driveController = driveMotor.getPIDController();
        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(Drivebase.METERS_PER_MOTOR_ROTATION);
        driveEncoder.setVelocityConversionFactor(Drivebase.METERS_PER_MOTOR_ROTATION / 60);
        driveEncoder.setAverageDepth(1);
        driveController.setP(Drivebase.VELOCITY_KP);
        driveController.setI(Drivebase.VELOCITY_KI);
        driveController.setD(Drivebase.VELOCITY_KD);
        driveController.setFF(Drivebase.VELOCITY_KF);
        driveController.setIZone(Drivebase.VELOCITY_IZ);
        driveMotor.setInverted(Drivebase.DRIVE_MOTOR_INVERT);
        driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        driveMotor.setSmartCurrentLimit(Drivebase.SWERVE_MODULE_CURRENT_LIMIT);

        driveMotor.burnFlash();
        angleMotor.burnFlash();

        feedforward = new SimpleMotorFeedforward(Drivebase.KS, Drivebase.KV, Drivebase.KA);

        time = new Timer();
        time.start();
        lastTime = time.get();
        
        lastAngle = getState().angle.getDegrees();
    }

    public void setDesiredState(BetterSwerveModuleState desiredState, boolean isOpenLoop) {
        setDesiredState(desiredState, isOpenLoop, true);
    }

    /*public void setGains(double kp, double ki, double kd, double ks, double kv, double ka) {
        feedforward = new SimpleMotorFeedforward(ks, kv, ka);
        driveController.setP(kp);
        driveController.setI(ki);
        driveController.setD(kd);
    }*/
    
    public void setDesiredState(BetterSwerveModuleState desiredState, boolean isOpenLoop, boolean antijitter) {
        SwerveModuleState simpleState = new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
        simpleState = SwerveModuleState.optimize(simpleState, getState().angle);
        desiredState = new BetterSwerveModuleState(simpleState.speedMetersPerSecond, simpleState.angle, desiredState.omegaRadPerSecond);
        SmartDashboard.putNumber("Optimized " + moduleNumber + " Speed Setpoint: ", desiredState.speedMetersPerSecond);
        SmartDashboard.putNumber("Optimized " + moduleNumber + " Angle Setpoint: ", desiredState.angle.getDegrees());
        SmartDashboard.putNumber("Module " + moduleNumber + " Omega: ", Math.toDegrees(desiredState.omegaRadPerSecond));

        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Drivebase.MAX_SPEED;
            driveMotor.set(percentOutput);
        } else {
            double velocity = desiredState.speedMetersPerSecond;
            driveController.setReference(velocity, ControlType.kVelocity, 0, feedforward.calculate(velocity));
        }

        double angle = ((Math.abs(desiredState.speedMetersPerSecond) <= (Drivebase.MAX_SPEED * 0.01)) && antijitter ? 
            lastAngle :
            desiredState.angle.getDegrees()); // Prevents module rotation if speed is less than 1%
        angleController.setReference(angle, ControlType.kPosition, 0, Math.toDegrees(desiredState.omegaRadPerSecond) * Drivebase.MODULE_KV);
        lastAngle = angle;

        this.angle = desiredState.angle.getDegrees();
        omega = desiredState.omegaRadPerSecond;
        speed = desiredState.speedMetersPerSecond;

        if (!Robot.isReal()) {
            dt = time.get() - lastTime;
            fakePos += (speed * dt);
            lastTime = time.get();
        }
    }

    public BetterSwerveModuleState getState() {
        double velocity;
        Rotation2d azimuth;
        double omega;
        if (Robot.isReal()) {
            velocity = driveEncoder.getVelocity();
            azimuth = Rotation2d.fromDegrees(absoluteEncoder.getPosition());
            omega = absoluteEncoder.getVelocity();
        } else {
            velocity = speed;
            azimuth = Rotation2d.fromDegrees(this.angle);
            omega = this.omega;
        }
        return new BetterSwerveModuleState(velocity, azimuth, omega);
    }

    public SwerveModulePosition getPosition() {
        double position;
        Rotation2d azimuth;
        if (Robot.isReal()) {
            position = driveEncoder.getPosition();
            azimuth = Rotation2d.fromDegrees(absoluteEncoder.getPosition());
        } else {
            position = fakePos;
            azimuth = Rotation2d.fromDegrees(angle + (Math.toDegrees(omega) * dt));
        }
        SmartDashboard.putNumber("Module " + moduleNumber + "Angle", azimuth.getDegrees());
        return new SwerveModulePosition(position, azimuth);
    }

    public double getAbsoluteEncoder() {
        return absoluteEncoder.getPosition();
    }

    public void setMotorBrake(boolean brake) {
        driveMotor.setIdleMode(brake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
    }

    public void turnModule(double speed) {
        angleMotor.set(speed);
        SmartDashboard.putNumber("AbsoluteEncoder" + moduleNumber, absoluteEncoder.getVelocity());
        SmartDashboard.putNumber("ControlEffort" + moduleNumber, angleMotor.getAppliedOutput());
    }
}
