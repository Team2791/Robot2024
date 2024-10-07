// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robotkt.constants.IOConstants;

public class Led extends SubsystemBase {
    public static AddressableLEDBuffer buf;
    public static AddressableLED led;

    public Led() {
        led = new AddressableLED(IOConstants.Led.kPort);
        buf = new AddressableLEDBuffer(IOConstants.Led.kLength);

        led.setLength(IOConstants.Led.kLength);
        applyBuffer();
    }

    public void applyBuffer() {
        led.setData(buf);
        led.start();
    }

    public void setRGB(int r, int g, int b) {
        for (var i = 0; i < buf.getLength(); i++) {
            buf.setRGB(i, r, g, b);
        }

        applyBuffer();
    }
}