; Simple asteroids demo with moving ship
; Memory addresses 2000+ are mapped to video
; TODO(Alin): Implement proper segmentation to access real video memory at 0xB8000
; See: https://wiki.osdev.org/Printing_To_Screen

bits 16
org 0

;<start>
    ; Draw title "ASTEROIDS" at row 5, col 35
    ; Position = (5 * 80) + 35 + 2000 = 2435
    MOV AL, 'A'
    MOV [2435], AL
    MOV AL, 'S'
    MOV [2436], AL
    MOV AL, 'T'
    MOV [2437], AL
    MOV AL, 'E'
    MOV [2438], AL
    MOV AL, 'R'
    MOV [2439], AL
    MOV AL, 'O'
    MOV [2440], AL
    MOV AL, 'I'
    MOV [2441], AL
    MOV AL, 'D'
    MOV [2442], AL
    MOV AL, 'S'
    MOV [2443], AL
    
    ; Draw some asteroids
    MOV AL, '*'
    MOV [2150], AL   ; Row 1, col 70 (1*80 + 70 + 2000)
    MOV [2090], AL   ; Row 1, col 10 
    MOV [2250], AL   ; Row 2, col 10
    MOV [2350], AL   ; Row 3, col 30
    MOV [2650], AL   ; Row 6, col 10
    MOV [2790], AL   ; Row 8, col 70
    MOV [2950], AL   ; Row 10, col 30
    MOV [3090], AL   ; Row 11, col 10
    
    ; Initialize ship position (row 17, starting at column 0)
    MOV BX, 3360    ; Row 17 * 80 + 0 + 2000 = 3360 (17*80=1360, +2000=3360)
    MOV CX, 0       ; Column counter (0-79)
    
game_loop:
    ; Clear previous ship position
    MOV AL, ' '
    MOV [BX], AL
    
    ; TODO(Alin): Change direction based on user input
    ; Could check memory-mapped keyboard at address 4000
    ; MOV AL, [4000]  ; Read last key pressed
    ; CMP AL, 'A'     ; Left arrow or 'A' key
    ; JE move_left
    ; CMP AL, 'D'     ; Right arrow or 'D' key  
    ; JE move_right
    ; Move to next column
    INC CX
    INC BX
    
    ; Check if we've reached the right edge (column 80)
    CMP CX, 80
    JL draw_ship    ; If less than 80, continue
    
    ; Wrap around to start
    MOV CX, 0
    MOV BX, 3360    ; Reset to start of row 17
    
draw_ship:
    ; TODO(Alin): Collision detection with asteroids
    ; Draw ship at new position
    MOV AL, '^'
    MOV [BX], AL
    
    ; TODO(Alin): Find a better way to handle delays
    ; Simple delay loop
    MOV SI, 65535   ; Maximum delay counter for very slow movement
delay_loop:
    DEC SI
    CMP SI, 0
    JNE delay_loop
    
    ; Continue the game loop
    JMP game_loop
;<end>