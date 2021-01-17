# Romi Arm Example

## Description
This example showcases how to use a [Romi Arm Kit](https://www.pololu.com/product/3550). You can control the arm using various buttons on a gamepad.

As the servos draw a significant amount of current, use of an external voltage regulator (operating off battery voltage) is required.

## Additional Hardware Required
- [Romi Arm Kit](https://www.pololu.com/product/3550)
- A 5V regulator like [this one](https://www.adafruit.com/product/1385)

## Additional Configuration Required
### Assembling the Romi Arm
Follow the instructions on the [Pololu website](https://www.pololu.com/docs/0J76/all)

### Setting up the voltage regulator input
The voltage regulator INPUT needs to be connected to the `VSW` pin and ground.

![VSW](doc-resources/romi-vsw.png)

### Setting up the voltage regulator output
The voltage regulator OUTPUT should be connected to the `GND` and power bus pins located next to the GPIO bank.

![Power Bus](doc-resources/romi-power-bus.png)

### Configuring GPIO pins
3 of the GPIO pins need to be configured as PWM outputs. 1 of the GPIO pins needs to be configured as an analog input.
- PWM 2 should be connected to the Gripper servo
- PWM 3 should be connected to the Tilt servo
- PWM 4 should be connected to the Lift servo
- AnalogInput 0 should be connected to the Gripper feedback wire

## Additional Code Setup
None