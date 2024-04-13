; Sum numbers from 1 to 5: 1+2+3+4+5 = 15
; Assemble with: nasm -f bin calc_sum_loop.asm -o calc_sum_loop.bin

bits 16
org 0

;<start>
    MOV AX, 0
    MOV CX, 1
    MOV DX, 5
    MOV BX, 1
loop_start:
    ADD AX, CX
    ADD CX, BX
    CMP CX, DX
    JLE loop_start
;<end>