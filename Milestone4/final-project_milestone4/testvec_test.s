.text
.align 4
main:

addi $3, $0, 0x800
ori  $4, $0, 0xF
lui  $5, 0xFFF
ori  $6, $0, 0xF
ori  $7, $0, 0x8
ori  $8, $0, 0x7
addi $9, $3, 0x4

forever:
	j forever
