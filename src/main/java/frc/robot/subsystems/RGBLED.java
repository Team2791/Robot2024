package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RGBLED extends SubsystemBase {
	public static AddressableLEDBuffer rBuffer;
	public static AddressableLED rgbled;
	public static int[][] rainbowRgbCombos;
	public static double colorPos = 0;
	public static int ledLength = 60;
	public static double shootPos = 0;
	public static double shootPos2;
	public static double animationSpeed = 0.5; // 1 == 1 robotperiodic call
	public static int rainbowDistanceMultiplier = 10;

	/** Creates a new RGBLED. */
	public RGBLED() {
		rgbled = new AddressableLED(9);
		rBuffer = new AddressableLEDBuffer(ledLength);
		rgbled.setLength(rBuffer.getLength());
		rgbled.setData(rBuffer);
		rgbled.start();
		// shootPos2 = rBuffer.getLength() / 2;

		rgbled.setData(rBuffer);

		rainbowRgbCombos = new int[768][3];

		for (int i = 0; i <= 255; i++) {
			int[] currentRgb = {
			    i, 0, 255 - i
			};
			rainbowRgbCombos[i] = currentRgb;
		}

		for (int i = 0; i <= 255; i++) {
			int[] currentRgb = {
			    255 - i, i, 0
			};
			rainbowRgbCombos[i + 256] = currentRgb;
		}

		for (int i = 0; i <= 255; i++) {
			int[] currentRgb = {
			    0, 255 - i, i
			};
			rainbowRgbCombos[i + 256 + 256] = currentRgb;
		}
	}

	public void setColor(int r, int g, int b) {
		for (var i = 0; i < 60; i++) {
			// Sets the specified LED to the RGB values for red
			rBuffer.setRGB(i, r, g, b);
		}
		// for (var i = 30; i < 45; i++) {
		// // Sets the specified LED to the RGB values for red
		// rBuffer.setRGB(i, r, g, b);
		// }
		rgbled.setData(rBuffer);
	}

	public void setSolid() {
		for (var i = 0; i < 15; i++) {
			rBuffer.setRGB((i), 0, 128, 128);
		}
		for (var i = 15; i < 45; i++) {
			rBuffer.setRGB((i), 255, 165, 0);
		}

		for (var i = 45; i < 60; i++) {
			rBuffer.setRGB((i), 0, 128, 128);
		}
		rgbled.setData(rBuffer);
	}

	public void setMode(String mode) {
		AddressableLEDBuffer buffer = new AddressableLEDBuffer(ledLength);

		if (mode == "rainbow") {
			for (int i = 0; i < ledLength; i++) {
				int[] currentRGB = rainbowRgbCombos[((int) Math.floor(
				    colorPos
				) + (i * rainbowDistanceMultiplier)) % (rainbowRgbCombos.length - 1)];

				buffer.setRGB(i, currentRGB[0], currentRGB[1], currentRGB[2]);

			}

			colorPos = (colorPos + (20 * animationSpeed)) % rainbowRgbCombos.length;
		} else if (mode == "shoot") {
			buffer.setRGB((int) Math.floor(shootPos), 0, 0, 255);
			buffer.setRGB((int) Math.floor(shootPos2), 0, 0, 255);

			shootPos = (shootPos + 1) % (ledLength / 2);
			shootPos2 = (shootPos2 + 1) % (ledLength);
		} else if (mode == "rainbowshoot") {
			int step = rainbowRgbCombos.length / ledLength;

			int index1 = step * (int) Math.floor(shootPos);
			int index2 = step * (((int) Math.floor(shootPos) + 1) % ledLength);
			int index3 = step * (((int) Math.floor(shootPos) + 2) % ledLength);
			int index4 = step * (((int) Math.floor(shootPos) + 3) % ledLength);
			int index5 = step * (((int) Math.floor(shootPos) + 4) % ledLength);

			int[] currentRGB1 = rainbowRgbCombos[index1];
			int[] currentRGB2 = rainbowRgbCombos[index2];
			int[] currentRGB3 = rainbowRgbCombos[index3];
			int[] currentRGB4 = rainbowRgbCombos[index4];
			int[] currentRGB5 = rainbowRgbCombos[index5];

			buffer.setRGB((int) Math.floor(shootPos), currentRGB1[0], currentRGB1[1], currentRGB1[2]);
			buffer.setRGB(
			    (((int) Math.floor(shootPos) + 1) % ledLength),
			    currentRGB2[0],
			    currentRGB2[1],
			    currentRGB2[2]
			);
			buffer.setRGB(
			    (((int) Math.floor(shootPos) + 2) % ledLength),
			    currentRGB3[0],
			    currentRGB3[1],
			    currentRGB3[2]
			);
			buffer.setRGB(
			    (((int) Math.floor(shootPos) + 3) % ledLength),
			    currentRGB4[0],
			    currentRGB4[1],
			    currentRGB4[2]
			);
			buffer.setRGB(
			    (((int) Math.floor(shootPos) + 4) % ledLength),
			    currentRGB5[0],
			    currentRGB5[1],
			    currentRGB5[2]
			);

			shootPos = (shootPos + 1) % ledLength;
		} else {
			mode = "rainbow";
		}
		rgbled.setData(buffer);

	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}

// UNFINISHED CODE
// package frc.robot.subsystems;

// import java.util.Optional;
// import java.util.stream.Stream;

// import edu.wpi.first.wpilibj.AddressableLED;
// import edu.wpi.first.wpilibj.AddressableLEDBuffer;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class RGBLED extends SubsystemBase {
// 	public static class RGBColor {
// 		final int r;
// 		final int g;
// 		final int b;

// 		public RGBColor(int r, int g, int b) {
// 			this.r = r;
// 			this.g = g;
// 			this.b = b;
// 		}
// 	}

// 	public static class SetRGB {
// 		final RGBMode mode;
// 		final Optional<RGBColor> solid;

// 		public SetRGB(RGBMode mode) throws IllegalArgumentException {
// 			this(mode, Optional.empty());

// 			if (mode == RGBMode.SolidColor) {
// 				throw new IllegalArgumentException("Solid color without solid color");
// 			}
// 		}

// 		public SetRGB(RGBColor solid) {
// 			this(RGBMode.SolidColor, Optional.of(solid));
// 		}

// 		private SetRGB(RGBMode mode, Optional<RGBColor> solid) {
// 			this.mode = mode;
// 			this.solid = solid;
// 		}
// 	}

// 	public static enum RGBMode {
// 		Rainbow, Shoot, RainbowShoot, SolidColor, Segmented, SolidRainbow;
// 	}

// 	public final int length = 60;

// 	private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(length);
// 	private final AddressableLED led = new AddressableLED(9);
// 	private final RGBColor[] colors = new RGBColor[768];

// 	public static double colorPos = 0;
// 	public static double shootPos = 0;
// 	public static double shootPos2;
// 	public static double animationSpeed = 0.5; // 1 == 1 robotperiodic call
// 	public static int rainbowDistanceMultiplier = 10;
// 	public static int shootHoleSize = 10;
// 	public static int shootAllowSize = 3;

// 	private SetRGB set = new SetRGB(RGBMode.Rainbow);

// 	/** Creates a new RGBLED. */
// 	public RGBLED() {
// 		led.setLength(buffer.getLength());
// 		led.setData(buffer);
// 		led.start();

// 		for (int i = 0; i <= 255; i++) colors[i] = new RGBColor(i, 0, 255 - i);
// 		for (int i = 0; i <= 255; i++) colors[i + 256] = new RGBColor(255 - i, i, 0);
// 		for (int i = 0; i <= 255; i++) colors[i + 256 + 256] = new RGBColor(0, 255 - i, i);
// 	}

// 	public void set(SetRGB set) {
// 		int step = (256 * 3) / length;

// 		switch (set.mode) {
// 			case RainbowShoot:
// 				for (int i = 0; i < colors.length; i += step) {
// 					RGBColor color = colors[i];
// 					buffer.setRGB(i, color.r, color.g, color.b);
// 				}

// 				break;
// 			case Rainbow:
// 				for (int i = 0; i < colors.length; i += step) {
// 					RGBColor color = colors[i];
// 					buffer.setRGB(i, color.r, color.g, color.b);
// 				}

// 				int prev = 0;
// 				for (int i = 0; i < 60; i++) {
// 					if (prev+)
// 				}

// 				break;
// 			case Segmented:
// 				break;
// 			case Shoot:
// 				break;
// 			case SolidColor:
// 				break;
// 			case SolidRainbow:
// 				break;
// 			default:
// 				break;
// 		}
// 	}

// 	public void setMode(String mode) {
// 		AddressableLEDBuffer buffer = new AddressableLEDBuffer(length);

// 		if (mode == "rainbow") {
// 			for (int i = 0; i < length; i++) {
// 				int[] currentRGB = rainbowRgbCombos[((int) Math.floor(
// 				    colorPos
// 				) + (i * rainbowDistanceMultiplier)) % (rainbowRgbCombos.length - 1)];

// 				buffer.setRGB(i, currentRGB[0], currentRGB[1], currentRGB[2]);

// 			}

// 			colorPos = (colorPos + (20 * animationSpeed)) % rainbowRgbCombos.length;
// 		} else if (mode == "shoot") {
// 			buffer.setRGB((int) Math.floor(shootPos), 0, 0, 255);
// 			buffer.setRGB((int) Math.floor(shootPos2), 0, 0, 255);

// 			shootPos = (shootPos + 1) % (length / 2);
// 			shootPos2 = (shootPos2 + 1) % (length);
// 		} else if (mode == "rainbowshoot") {
// 			int step = rainbowRgbCombos.length / length;

// 			int index1 = step * (int) Math.floor(shootPos);
// 			int index2 = step * (((int) Math.floor(shootPos) + 1) % length);
// 			int index3 = step * (((int) Math.floor(shootPos) + 2) % length);
// 			int index4 = step * (((int) Math.floor(shootPos) + 3) % length);
// 			int index5 = step * (((int) Math.floor(shootPos) + 4) % length);

// 			int[] currentRGB1 = rainbowRgbCombos[index1];
// 			int[] currentRGB2 = rainbowRgbCombos[index2];
// 			int[] currentRGB3 = rainbowRgbCombos[index3];
// 			int[] currentRGB4 = rainbowRgbCombos[index4];
// 			int[] currentRGB5 = rainbowRgbCombos[index5];

// 			buffer.setRGB((int) Math.floor(shootPos), currentRGB1[0], currentRGB1[1], currentRGB1[2]);
// 			buffer.setRGB((((int) Math.floor(shootPos) + 1) % length), currentRGB2[0], currentRGB2[1], currentRGB2[2]);
// 			buffer.setRGB((((int) Math.floor(shootPos) + 2) % length), currentRGB3[0], currentRGB3[1], currentRGB3[2]);
// 			buffer.setRGB((((int) Math.floor(shootPos) + 3) % length), currentRGB4[0], currentRGB4[1], currentRGB4[2]);
// 			buffer.setRGB((((int) Math.floor(shootPos) + 4) % length), currentRGB5[0], currentRGB5[1], currentRGB5[2]);

// 			shootPos = (shootPos + 1) % length;
// 		} else {
// 			mode = "rainbow";
// 		}
// 		led.setData(buffer);

// 	}

// 	@Override
// 	public void periodic() {

// 	}
// }