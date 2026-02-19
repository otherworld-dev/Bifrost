; homey.g
; called to home the Y axis
;

G91                      ; relative positioning
G1 H1 Y180 F1800        ; move quickly to Y axis endstop and stop there (first pass)
G1 H2 Y-5 F6000           ; go back a few degrees
G1 H1 Y180 F360         ; move slowly to Y axis endstop once more (second pass)
G90                      ; absolute positioning
