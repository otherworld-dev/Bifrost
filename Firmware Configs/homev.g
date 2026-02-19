; homev.g
; called to home the V axis
;

G91                      ; relative positioning
;G1 H1 V-360 W360 F1800        ; move quickly to V axis endstop and stop there (first pass)
;G1 H2 V1 W-1 F6000           ; go back a few degrees
;G1 H1 V-360 W360 F360         ; move slowly to V axis endstop once more (second pass)

G1 H1 V360 W-360 F1800        ; move quickly to V axis endstop and stop there (first pass)
G1 H2 V-1 W1 F6000           ; go back a few degrees
G1 H1 V360 W-360 F360         ; move slowly to V axis endstop once more (second pass)

G90                      ; absolute positioning
