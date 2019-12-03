.text
.align 4

# a1 = i, a2 = j
# t2 = # of data (bit)
main:
    la   $t0, Input_data   # t0 = base address of Input_data
    la   $t1, Output_data  # t1 = base address of Output_data

    sub  $t2, $t1, $t0     # count = 32

    lw   $a0, 0($t0)
    sw   $a0, 0($t1)       # output_arr[0] = input_arr[0]

    addi $a1, $zero, 4     # i = 1 

Loop1: # loop I
    bge  $a1, $t2, exit    # if i >= count, then END
    addi $a2, $a1, -4      # j = i - 1
    
    add  $t3, $t0, $a1     # &input_arr[i]
    add  $t4, $t1, $a1     # &output_arr[i]

    lw   $a3, 0($t3)
    sw   $a3, 0($t4)       # output_arr[i] = input_arr[i]

Loop2: # loop J
    blt  $a2, $zero, end   # if j < 0, then END
    
    add  $s2, $t1, $a2     # &arr[j]
    add  $s3, $s2, 4       # &arr[j+1]

    lw   $s0, 0($s2)       # arr[j]
    lw   $s1, 0($s3)       # arr[j+1]

    bge  $s0, $s1, end     # if arr[j] >= arr[j+1], then END

    sw   $s1, 0($s2)       # swap arr[j], arr[j+1]
    sw   $s0, 0($s3)

    addi $a2, $a2, -4      # j--

    j    Loop2

end:
    addi $a1, $a1, 4       # i++
    j    Loop1

exit:
    j    exit


.data
.align 4
Input_data: .word 2, 0, -7, -1, 3, 8, -4, 10
            .word -9, -16, 15, 13, 1, 4, -3, 14
            .word -8, -10, -15, 6, -13, -5, 9, 12
            .word -11, -14, -6, 11, 5, 7, -2, -12

Output_data: .word 0, 0, 0, 0, 0, 0, 0, 0
             .word 0, 0, 0, 0, 0, 0, 0, 0
             .word 0, 0, 0, 0, 0, 0, 0, 0
             .word 0, 0, 0, 0, 0, 0, 0, 0
