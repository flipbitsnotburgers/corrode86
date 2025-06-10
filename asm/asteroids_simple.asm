; Simple asteroids demo with moving ship
; Uses video memory at 0xB8000 (standard text mode location)
; Text mode format: each character is 2 bytes (char + attribute)
; Physical address 0xB8000 = segment 0xB800, offset 0x0000

bits 16
org 0

;<start>
    ; Set ES to point to video memory (text mode)
    MOV AX, 0xB800
    MOV ES, AX
    
    ; Draw title "ASTEROIDS" at row 5, col 35
    ; Position = ((5 * 80) + 35) * 2 = 870
    ; We multiply by 2 because each character takes 2 bytes
    MOV AL, 'A'
    MOV [ES:870], AL
    MOV AL, 'S'
    MOV [ES:872], AL
    MOV AL, 'T'
    MOV [ES:874], AL
    MOV AL, 'E'
    MOV [ES:876], AL
    MOV AL, 'R'
    MOV [ES:878], AL
    MOV AL, 'O'
    MOV [ES:880], AL
    MOV AL, 'I'
    MOV [ES:882], AL
    MOV AL, 'D'
    MOV [ES:884], AL
    MOV AL, 'S'
    MOV [ES:886], AL
    
    ; Draw some asteroids
    MOV AL, '*'
    MOV [ES:300], AL   ; Row 1, col 70: (1*80 + 70)*2 = 300
    MOV [ES:180], AL   ; Row 1, col 10: (1*80 + 10)*2 = 180
    MOV [ES:340], AL   ; Row 2, col 10: (2*80 + 10)*2 = 340
    MOV [ES:540], AL   ; Row 3, col 30: (3*80 + 30)*2 = 540
    MOV [ES:980], AL   ; Row 6, col 10: (6*80 + 10)*2 = 980
    MOV [ES:1420], AL  ; Row 8, col 70: (8*80 + 70)*2 = 1420
    MOV [ES:1660], AL  ; Row 10, col 30: (10*80 + 30)*2 = 1660
    MOV [ES:1780], AL  ; Row 11, col 10: (11*80 + 10)*2 = 1780
    
    ; Initialize ship position (row 17, starting at column 0)
    MOV BX, 2720    ; Row 17: 17*80*2 = 2720
    MOV CX, 0       ; Column counter (0-79)
    
game_loop:
    ; Clear previous ship position
    MOV AL, ' '
    MOV [ES:BX], AL
    
    ; TODO(Alin): Change direction based on user input
    ; Could check memory-mapped keyboard at address 4000
    ; MOV AL, [4000]  ; Read last key pressed
    ; CMP AL, 'A'     ; Left arrow or 'A' key
    ; JE move_left
    ; CMP AL, 'D'     ; Right arrow or 'D' key  
    ; JE move_right
    ; Move to next column (skip attribute byte)
    INC CX
    ADD BX, 2       ; Move 2 bytes (char + attribute)
    
    ; Check if we've reached the right edge (column 80)
    CMP CX, 80
    JL draw_ship    ; If less than 80, continue
    
    ; Wrap around to start
    MOV CX, 0
    MOV BX, 2720    ; Reset to start of row 17
    
draw_ship:
    ; TODO(Alin): Collision detection with asteroids
    ; Draw ship at new position
    MOV AL, '^'
    MOV [ES:BX], AL
    
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