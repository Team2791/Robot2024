package frc.robot.subsystems;

import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import java.util.stream.Stream;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Led extends SubsystemBase {
	public static class LedMode {
		Optional<Color> base;
		boolean segmented;
		boolean rotating;

		private LedMode() {
			this.base = Optional.empty();
			this.segmented = false;
			this.rotating = false;
		}

		public static LedMode init() {
			return new LedMode();
		}

		public LedMode base(Color color) {
			this.base = Optional.of(color);
			return this;
		}

		public LedMode segmented(boolean segmented) {
			this.segmented = segmented;
			return this;
		}

		/** Also turns on rotation */
		public LedMode segmented() {
			rotating();
			return segmented(true);
		}

		public LedMode rotating(boolean rotating) {
			this.rotating = rotating;
			return this;
		}

		public LedMode rotating() {
			return rotating(true);
		}

		public void setInitialBuffer(AddressableLEDBuffer buffer) {
			if (this.base.isPresent()) {
				Color color = this.base.get();

				for (int i = 0; i < Constants.Led.Length; i++) {
					buffer.setLED(i, color);
				}
			} else {
				int j = 0;
				int i = 0;

				for (; j <= 255; j += Constants.Led.RainbowStep) buffer.setRGB(i++, j, 0, 255 - j);
				for (; j <= 510; j += Constants.Led.RainbowStep) buffer.setRGB(i++, 510 - j, j - 255, 0);
				for (; j <= 765; j += Constants.Led.RainbowStep) buffer.setRGB(i++, 0, 765 - j, j - 510);

				// fill the rest with less-rapidly changing colors
				while (i != Constants.Led.Length) buffer.setRGB(i++, 0, Math.max(0, 765 - j), Math.min(255, j++ - 510));
			}

			if (this.segmented) {
				int i = 0;

				while (i < Constants.Led.Length) {
					i += Constants.Led.ShootSize;
					for (int j = 0; j < Constants.Led.ShootSizeHole; j++) buffer.setLED(i++, Color.kBlack);
				}
			}
		}
	}

	private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(Constants.Led.Length);
	private final AddressableLED led = new AddressableLED(9);

	private LedMode mode = LedMode.init().base(Color.kBlack);
	private double accumulator = 0;

	public Led() {
		apply();

		CommandScheduler.getInstance().registerSubsystem(this);
	}

	public void set(LedMode mode) {
		mode.setInitialBuffer(buffer);

		this.mode = mode;
		this.apply();
	}

	public List<Color> bufferToList() {
		return Stream.generate(new Supplier<Color>() {
			private int i = 0;

			public Color get() {
				return buffer.getLED(i++);
			}
		}).limit(buffer.getLength()).toList();
	}

	public void listToBuffer(List<Color> colors) {
		for (int i = 0; i < buffer.getLength(); i++) {
			buffer.setLED(i, colors.get(i));
		}
	}

	public void periodic() {
		if (!this.mode.rotating) return;

		this.accumulator += Constants.Led.RotatingSpeed;

		if (this.accumulator < 1) return;
		else this.accumulator = 0;

		List<Color> colors = bufferToList();
		Collections.rotate(colors, Constants.Led.ShiftSize);

		this.listToBuffer(colors);
		this.apply();
	}

	public void apply() {
		led.setData(buffer);
		led.start();
	}
}