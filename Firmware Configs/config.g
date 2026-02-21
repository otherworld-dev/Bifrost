; Configuration file for BTT Octopus Pro v1.0 F429 (firmware version 3.4.x)
; executed by the firmware on start-up
;
; Migrated from Fly Super8 Pro H723 config
; For use with Thor robot arm

; General preferences
G90                                                                      ; send absolute coordinates...
M83                                                                      ; ...but relative extruder moves
M550 P"Thor"                                                             ; set printer name

; Network
;M552 S1                                                                  ; enable network
;M586 P0 S1                                                               ; enable HTTP
;M586 P1 S0                                                               ; disable FTP
;M586 P2 S0                                                               ; disable Telnet

; Drives
M569 P0 S0 T1.0:1.0:0.2:0.2                                              ; physical drive 0 goes forwards using A4988 driver timings
M569 P1 S0 T1.0:1.0:0.2:0.2                                              ; physical drive 1 goes forwards using A4988 driver timings
M569 P2 S0 T1.0:1.0:0.2:0.2                                              ; physical drive 2 goes forwards using A4988 driver timings
M569 P3 S0 T1.0:1.0:0.2:0.2                                              ; physical drive 3 goes forwards using A4988 driver timings
M569 P4 S1 T1.0:1.0:0.2:0.2                                              ; physical drive 4 goes forwards using A4988 driver timings
M569 P5 S1 T1.0:1.0:0.2:0.2                                              ; physical drive 5 goes forwards using A4988 driver timings
M569 P6 S1 T1.0:1.0:0.2:0.2                                              ; physical drive 6 goes forwards using A4988 driver timings
M584 X0 Y1:2 Z3 U4 V5 W6                                                 ; set drive mapping
M350 X16 Y16 Z16 U16 V16 W16 I0                                          ; configure microstepping (no interpolation - A4988 drivers)
M92 X44.50 Y265.00 Z265.00 U17.55 V36 W36                           ; set steps per mm (Y matches Z: same 30:1 gear reduction)
M566 X900.00 Y900.00 Z900.00 U900.00 V9000.00 W9000.00                   ; set maximum instantaneous speed changes (mm/min)
M203 X2000.00 Y800.00 Z800.00 U2000.00 V5000.00 W5000.00                 ; set maximum speeds (mm/min)
M201 X10.00 Y10.00 Z10.00 U10.00 V100.00 W100.00                         ; set accelerations (mm/s^2)
;M906 X800 Y800 Z800 U800 V800 W800 I30                                  ; set motor currents (mA) and motor idle factor in per cent
M84 S30                                                                  ; Set idle timeout

; Servomotor	
M950 S0 C"PB_6"                                                          ; Servo on PROBE header (J40)

; Axis Limits
M208 X-97 Y-90 Z-90 U-180 V-200 W-200 S1                                 ; set axis minima
M208 X97 Y90 Z90 U180 V200 W200 S0                                       ; set axis maxima
M564 S0                                                                  ; allow movement outside boundaries

; Endstops
; Pin mapping: io0-io6 (Super8) -> PG_6,PG_9,PG_10,PG_11,PG_12,PG_13,PG_14 (Octopus Pro)
M574 X1 S1 P"!PG_6"                                                      ; X endstop on STOP0 (J27) - inverted
M574 Y1 S1 P"!PG_9"                                                      ; Y endstop on STOP1 (J29) - inverted
M574 Z1 S1 P"!PG_10"                                                     ; Z endstop on STOP2 (J31) - inverted
M574 U1 S1 P"!PG_11"                                                     ; U endstop on STOP3 (J33) - inverted
M574 V1 S1 P"!PG_12"                                                      ; V endstop on STOP4 (J28)
M574 W1 S1 P"PG_13"                                                     ; W endstop on STOP5 (J30) - inverted (no physical switch)

; Z-Probe
M558 P0 H5 F120 T6000                                                    ; disable Z probe but set dive height, probe speed and travel speed
M557 X15:215 Y15:195 S20                                                 ; define mesh grid

; Heaters
M140 H-1                                                                 ; disable heated bed (overrides default heater mapping)

; Robot Fans
M950 F0 C"PA_8" Q500                                                     ; FAN0 (J50)
M106 P0 S1 H-1                                                           ; set fan 0 value. Thermostatic control is turned off
M950 F1 C"PE_5" Q500                                                     ; FAN1 (J51)
M106 P1 S1 H-1                                                           ; set fan 1 value. Thermostatic control is turned off
M950 F2 C"PD_12" Q500                                                    ; FAN2 (J52)
M106 P2 S1 H-1                                                           ; set fan 2 value. Thermostatic control is turned off
M950 F3 C"PD_13" Q500                                                    ; FAN3 (J53)
M106 P3 S1 H-1                                                           ; set fan 3 value. Thermostatic control is turned off
M950 F4 C"PD_14" Q500                                                    ; FAN4 (J54)
M106 P4 S1 H-1                                                           ; set fan 4 value. Thermostatic control is turned off
M950 F5 C"PD_15" Q500                                                    ; FAN5 (J55)
M106 P5 S1 H-1                                                           ; set fan 5 value. Thermostatic control is turned off

; Electronic Fans (FAN6/FAN7 are always-on, no PWM - only 6 PWM fans available)
; If you need more PWM fans, use heater output pins

; Tools
M563 P0 D0 F0                                                            ; define tool 0
G10 P0 X0 Y0 Z0                                                          ; set tool 0 axis offsets
G10 P0 R0 S0                                                             ; set initial tool 0 active and standby temperatures to 0C

; Input/Output
M950 J1 C"PG_14"                                                         ; Input 1 on STOP6 (J32) - Home button
M581 P1 T3 S1 R0                                                         ; Home All - trigger3.g

; Custom settings are not defined
;M208 X194 Y90 Z90 U170 V200 W200 S0 
;M208 X-97:97 Y-90:+90 Z-90:90 U-180:180 V-200:200 W-200:200 S0; DISABLED - colon syntax conflicts with S0 flag, use separate S0/S1 above