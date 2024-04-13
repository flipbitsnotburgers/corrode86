; Simple addition: 10 + 20 = 30
; Assemble with: nasm -f bin calc_add.asm -o calc_add.bin

bits 16
org 0

;<start>
    MOV AX, 10
    MOV BX, 20
    ADD AX, BX
;<end>