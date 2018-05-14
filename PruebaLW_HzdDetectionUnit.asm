.text
addi $s0, $zero, 5#5
addi $s1, $zero, 7#7
ori $t0, $t0, 0x1001 		# Loads higher bytes of memory address
sll $t0, $t0, 16     		# Moves higher bytes of memory to left
ori $s5, $t0, 0x0000 		# In
sw $s0,16($s5)



lw $v0,16($s5)            #5
add $zero,$zero, $zero
add $zero,$zero, $zero
add $zero,$zero, $zero
add $zero,$zero, $zero
add $zero,$zero, $zero

addi $t0, $s1, 5         #12
#sub $a0, $v0, $t0        #-5
#add $v0, $zero, $v0      #5

add $zero,$zero, $zero
add $zero,$zero, $zero
add $zero,$zero, $zero
add $zero,$zero, $zero
add $zero,$zero, $zero
add $zero,$zero, $zero

