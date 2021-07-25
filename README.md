# Microcontrollers
My programs for AVR attiny & atmega microcontrollers
# Electric Go-Kart
![20210725_212951](https://user-images.githubusercontent.com/37122127/126917548-9342f97b-d21c-4d45-8655-7d04908fee9e.jpg)
![20210725_212924](https://user-images.githubusercontent.com/37122127/126917552-bf20f786-02de-42e2-8639-851dd741d279.jpg)
## Specification:
* 2x250W DC motor
* 22Ah, 25.9V Li-Ion battery
* Self-made DC motor driver able to handle up to 200 Amps of current
* Top speed: 18km / h
* Range: ~ 14km
* Video: [YouTube](https://youtu.be/nwERPI5T8nM)

## Tasks performed by the gokart.c program
* Reading state of the gas pedal
* Reading state of the braking pedal
* Generating PWM waveforms for controlling electric motors
* Allowing smooth control of motors power from 0 to 100%
* Making the go-kart to decelerate slowly in case of quick release of the gas pedal
# Gun Chronograph
## Tasks performed by the chrono.c program
* Precise time measuring between triggering photocells by a bullet
* Calculating bullet's speed from known distance and time
* Displaying calculated speed on 8-segment displays in multiplex mode
