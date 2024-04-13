; Complex calculation: (100 + 25) - (30 + 10) = 85
; Assemble with: nasm -f bin calc_complex.asm -o calc_complex.bin

bits 16
org 0

;<start>
    MOV AX, 100
    ADD AX, 25
    MOV CX, AX
    MOV AX, 30
    ADD AX, 10
    MOV BX, AX
    MOV AX, CX
    SUB AX, BX
;<end>