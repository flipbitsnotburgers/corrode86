; Simple subtraction: 50 - 15 = 35
; Assemble with: nasm -f bin calc_sub.asm -o calc_sub.bin

bits 16
org 0

;<start>
    MOV AX, 50
    MOV BX, 15
    SUB AX, BX
;<end>