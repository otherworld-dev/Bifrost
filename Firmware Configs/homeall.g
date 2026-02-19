; homeall.g
; called to home all axes
;

M17                      ; Enable all steppers

; HOME X - Articulation 1
G91                      ; relative positioning
G1 H1 X-360 F1800        ; move quickly to X axis endstop and stop there (first pass)
G1 H2 X5 F6000           ; go back a few degrees
G1 H1 X-360 F360         ; move slowly to X axis endstop once more (second pass)
G90                      ; absolute positioning
G92 X0

; HOME Y - Articulation 2
G91                      ; relative positioning
G1 H1 Y180 F1800        ; move quickly to Y axis endstop and stop there (first pass)
G1 H2 Y-5 F6000           ; go back a few degrees
G1 H1 Y180 F360         ; move slowly to Y axis endstop once more (second pass)
G90                      ; absolute positioning
M206 Y90

; HOME Z - Articulation 3
G91                      ; relative positioning
G1 H1 Z-180 F1800        ; move quickly to Z axis endstop and stop there (first pass)
G1 H2 Z5 F6000           ; go back a few degrees
G1 H1 Z-180 F360         ; move slowly to Z axis endstop once more (second pass)
G90                      ; absolute positioning
M206 Z90

; HOME U - Articulation 4
G91                      ; relative positioning
G1 H1 U360 F1800        ; move quickly to U axis endstop and stop there (first pass)
G1 H2 U-5 F6000           ; go back a few degrees
G1 H1 U360 F360         ; move slowly to U axis endstop once more (second pass)
G90                      ; absolute positioning
G92 U0

; HOME V - Articulation 5
G91                           ; relative positioning
G1 H1 V360 W-360 F1800        ; move quickly to V axis endstop and stop there (first pass)
G1 H2 V-1 W1 F6000            ; go back a few degrees
G1 H1 V360 W-360 F360         ; move slowly to V axis endstop once more (second pass)
G90                           ; absolute positioning


G91
M564 H0 S0      ; disable limit enforcement
G1 V-90 F3000
M400
G92 V0
M564 H1 S1      ; re-enable limits
G90