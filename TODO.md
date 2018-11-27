- [ ] refactor okWatcher branch:  since we can't figure out which "ok" is for which command for jog's perhaps we should calculate the desired position and then wait until it is reached before clearing the "waiting" flag
- [ ] Add back encoder
- [ ] Add back position parsing
- [ ] tidy up msgs and display
- [ ] ensure buffer can't fill when running batchJog
- [ x ] wire up encoders and change PPR.

- [ X ] add OLED support
- [ X ] mock up hardware
- [ X ]  set timer when in jog mode to get updates more quickly
- [ X ] accel mode which uses wheel speed to set step size and speed =0 to issue CMD_JOG_CANCEL

- [ ] do some testing of echo and job start job end, I believe this may be "Run" mode


...  Have ISR call Axis object directly, move axis objects to global space.


rework command parsing.

"response messages" start with ok, or error.  they should inc and dec counters to track the buffer.

"push messages" are not counted towards streaming.  these start with [ and <

Realtime messages are ack'ed with "ok" also.  simple_stream.py maps commands to a FIFO buffer and takes buffer[0] off every time it see's an 'ok' response.

in order to work with the GUI we should return some error on serial if the MITM is in jog mode.  no counting should happen for passive mode and before changing modes there should be some wait to ensure the buffers are clear.

