/*
* func.s
*/
//.section .data
.syntax unified //as.pdf : p141
.align 16 //as.pdf :p71
.section .text, "x" //as.pdf :p96
//.rodata
.global func //as.pdf : p254
/**
Comments for code readability
*/
//Your assembly code
func: //label
//PUSH current state to stack
//setup registers
//your function
//POP stack
CMP R0, R1// CMP compares two numeric data fields
BLE else //Branch on Less than or Equal
MOV R2, R0 //set value
MOV R2, R0 //set value
B endif //branch
else: MOV R2, R1
endif:

BX LR
