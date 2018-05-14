.text
	addi $v0, $zero, 77
		
	beq $zero, $zero, salto1
	addi $t2, $zero, 4
salto1: 
	add $zero, $zero, $zero#Nop
	bne $v0, $zero, salto2
	addi $t2, $zero, 5
	
salto2:
	j salto3
	addi $t2, $zero, 6
	
salto3:
	jal function
	j exit

	
function:
	addi $v0, $zero, 88
	add $zero, $zero, $zero#Nop
	jr $ra
	
exit:	
	addi $v0, $zero, 99
	
	
	addi $t0, $zero, 5#5
	addi $t1, $t0, 10#15
	add $t2, $t0, $t1#20
	sub $t2, $t1, $t2#-5
	add $t3, $t2,$t2#-10	
	add $zero, $zero, $zero#Nop
	add $zero, $zero, $zero#Nop
	add $zero, $zero, $zero#Nop

