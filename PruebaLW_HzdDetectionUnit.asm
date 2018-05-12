.text
addi $s0, $zero, 5#5
ori $t0, $t0, 0x1001 		# Loads higher bytes of memory address
sll $t0, $t0, 16     		# Moves higher bytes of memory to left
ori $s5, $t0, 0x0000 		# In
sw $s0,0($s5)

add $zero,$zero, $zero
add $zero,$zero, $zero
add $zero,$zero, $zero
add $zero,$zero, $zero
add $zero,$zero, $zero

lw $v0,0($s5)            #5
addi $t0, $v0, 5         #10
sub $t2, $v0, $t0        #-5
add $t2, $zero, $v0      #5

