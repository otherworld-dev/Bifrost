; homez.g
; called to home the Z axis
;

G91                      ; relative positioning
G1 H1 Z-180 F1800        ; move quickly to Z axis endstop and stop there (first pass)
G1 H2 Z5 F6000           ; go back a few degrees
G1 H1 Z-180 F360         ; move slowly to Z axis endstop once more (second pass)
G90                      ; absolute positioning
