# Stm32 grbl-MITM-pendant

## forked from Simon Merrett's version
https://github.com/SimonMerrett/grbl-MITM-pendant


A basic arduino sketch that allows you to use a rotary encoder and other Arduino inputs, along with a separate Arduino, as jog controls for grbl CNC while maintaining a serial connection to a PC

<i> Bear in mind that this is a work in progress. Key features missing are use of the proper jog commands for grbl controllers - currently uses rapid movements; also missing is a reset of the Arduino running grbl once the pendant Arduino has established a serial connection with the PC gcode sender application. Some, i.e. Universal Gcode Sender, listen for the grbl initialisation message before allowing commands to be send.
  
  DO NOT USE WHEN PC IS SENDING GCODE FILES AND COMMANDS TO THE GRBL ARDUINO.</i>

## Status

working with 3 encoders and oled!

## Senders

For UGS it appears you need to find you ~/.ugs directory and make a copy of grbl.json and replace "GRBL" in the file with "XLCD".  This prevents error popups which hopelessly lock up UGS


# TODO list

- [ ] Add reset command to ensure we can deal with alarm conditions
- [ ] figure out what dedicated buttons are wanted
- [ ] Process job start job end and implement lockout
- [ ] Configuration for Jog speed
- [ ] add OLED screen support for stuff like DRO readings, maybe need one screen on the pendant and one on the mill
- [ ] rapid mode with hold down key
- [ ] DRO and go mode, enter distance pulled from grbl config and execute at speed when go key hit
- [ ] virtual stops, jog to and set or set from current position.  kinda like per job softlmits
- [ ] store config and variables for different encoder hardware, detents, etc
- [ ] raize Z, for bit change raise just to the verge of the Z stop.


probe modes

- [ ] half
- [ ] circle center from 3 points


semi automatic modes

- [ ] facing mode based on 4 virtual stops, stepover, and stepdown
- [ ] slotting mode

tweaks

- [ ] spindle speed %
- [ ] feed %
- [ ] other options available 

safety

- [ ] estop 
- [ ] pause

