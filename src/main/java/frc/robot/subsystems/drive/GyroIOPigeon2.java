// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon = new Pigeon2(20);
  private final StatusSignal<Double> yaw = pigeon.getYaw();
  private final StatusSignal<Double> yawVelocity = pigeon.getAngularVelocityZWorld();

  private final StatusSignal<Double> accelerationX = pigeon.getAccelerationX();
  private final StatusSignal<Double> accelerationY = pigeon.getAccelerationY();
  private final StatusSignal<Double> accelerationZ = pigeon.getAccelerationZ();

  private final StatusSignal<Double> getMagFieldX = pigeon.getMagneticFieldX();
  private final StatusSignal<Double> getMagFieldY = pigeon.getMagneticFieldY();
  private final StatusSignal<Double> getMagFieldZ = pigeon.getMagneticFieldZ();

  private final StatusSignal<Double> quatW = pigeon.getQuatW();
  private final StatusSignal<Double> quatX = pigeon.getQuatX();
  private final StatusSignal<Double> quatY = pigeon.getQuatY();
  private final StatusSignal<Double> quatZ = pigeon.getQuatZ();

  public GyroIOPigeon2() {
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);
    StatusSignal.setUpdateFrequencyForAll(
            100,
            yaw, yawVelocity, accelerationY, accelerationX, accelerationZ, getMagFieldX,
            getMagFieldY, getMagFieldZ,
            quatW, quatX, quatY, quatZ
    );
    pigeon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    StatusCode x = BaseStatusSignal.refreshAll(
            yaw, yawVelocity, accelerationX, accelerationY, accelerationZ,
            getMagFieldX, getMagFieldY, getMagFieldZ,
            quatW, quatX, quatY, quatZ
    );
    inputs.connected = x.equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
    inputs.accelX = accelerationX.getValueAsDouble();
    inputs.accelY = accelerationY.getValueAsDouble();
    inputs.accelZ = accelerationZ.getValueAsDouble();
    inputs.quatW = quatW.getValueAsDouble();
    inputs.quatX = quatX.getValueAsDouble();
    inputs.quatY = quatY.getValueAsDouble();
    inputs.quatZ = quatZ.getValueAsDouble();


  }
}
