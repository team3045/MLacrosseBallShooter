// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private TalonFX motor1Controller = new TalonFX(0);
  private TalonFX motor2Controller = new TalonFX(0);

  private double motor1Target = 0;
  private double motor2Target = 0;

  private double kP = 0.01;
  private double feedForward = 0.01;

  private final static double motorGearRed = 1.5;
  private final static double jKg = 0.001564; //value for 1 motor (1.1 in^2/lb): 0.00156456776 m^2/kg

  private final static FlywheelSim sim = new FlywheelSim(DCMotor.getFalcon500(1), motorGearRed, jKg);
  private final TalonFXSimState controller1Sim = new TalonFXSimState(motor1Controller);
  private final TalonFXSimState controller2Sim = new TalonFXSimState(motor2Controller);
  /** Creates a new Shooter. */
  public Shooter() {
    sim.setInputVoltage(RobotController.getBatteryVoltage());
    configMotors();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    VelocityVoltage request1 = new VelocityVoltage(motor1Target);
    VelocityVoltage request2 = new VelocityVoltage(motor2Target);
    motor1Controller.setControl(request1.withSlot(0));
    motor2Controller.setControl(request2.withSlot(0));
  }

  public void configMotors() {
    controller1Sim.setSupplyVoltage(RobotController.getBatteryVoltage());
    controller2Sim.setSupplyVoltage(RobotController.getBatteryVoltage());
    TalonFXConfiguration config = new TalonFXConfiguration().withSlot0(new Slot0Configs().withKP(kP).withKV(feedForward));
    motor1Controller.getConfigurator().apply(config);
    motor2Controller.getConfigurator().apply(config);
  }

  public void setMotor1Target(double targetSpeed) {
    motor1Target = targetSpeed;
  }

  public void setMotor2Target(double targetSpeed) {
    motor2Target = targetSpeed;
  }

  public double getMotor1Target() {
    return motor1Target;
  }

  public double getMotor2Target() {
    return motor2Target;
  }
}