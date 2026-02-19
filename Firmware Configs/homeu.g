; homeu.g
; called to home the U axis
;

G91                      ; relative positioning
G1 H1 U360 F1800        ; move quickly to U axis endstop and stop there (first pass)
G1 H2 U-5 F6000           ; go back a few degrees
G1 H1 U360 F360         ; move slowly to U axis endstop once more (second pass)
G90                      ; absolute positioning
