.text
	addi $v0, $zero, 77
		
	beq $zero, $zero, salto1
	addi $t2, $zero, 4
salto1: 
	addi $t0, $zero, 79
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
	add $zero, $zero, $zero
	jr $ra
	
exit:	
	addi $v0, $zero, 99
	add $zero, $zero, $zero
	add $zero, $zero, $zero
	add $zero, $zero, $zero

