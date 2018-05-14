.text
	addi $t0, $zero, 7
	jal function
	j exit
	addi $a0, $zero, 234
	
function:
	addi $v0, $zero, 8
	add $zero, $zero, $zero
	jr $ra
	
exit:
	addi $a0, $zero, 111
	add $zero, $zero, $zero
	add $zero, $zero, $zero
	add $zero, $zero, $zero

	