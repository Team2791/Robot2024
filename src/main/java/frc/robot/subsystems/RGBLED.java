// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class RGBLED extends SubsystemBase {
  /** Creates a new RGBLED. */
  public static AddressableLEDBuffer rBuffer;
  public static AddressableLED rgbled;
  public static int ledLength;
  Timer timer;

  public RGBLED() {

    timer = new Timer();
    rgbled = new AddressableLED(0);
    ledLength = 25;
    rBuffer = new AddressableLEDBuffer(ledLength);
    rgbled.setLength(rBuffer.getLength());
    rgbled.setData(rBuffer);
    rgbled.start();
    timer.reset();
    timer.start();
  }

  public void setColor(int r, int g, int b) {
    for (var i = 0; i < ledLength; i++) {
      rBuffer.setRGB(i, r, g, b);
    }
    rgbled.setData(rBuffer);
  }

  public void delay(Timer timer, double s){
    double current = timer.get();
    while(timer.get()<current+s)
    {

    }
    return;
  }


  public void flash(int r, int g, int b, double time){
    double current = timer.get();
    while(timer.get()<current + time)
    {
      setColor(255, 255, 255);
      delay(timer, 0.5);
      setColor(0, 0, 0);
      delay(timer, 0.5);
    }
    return;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //if(Robot.shintake.isin())flash(0,255,0,3);
  }
}