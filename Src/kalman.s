
// kalman.s

//.section .data
.syntax unified //as.pdf : p141
.align 16 //as.pdf :p71
.section .text, "x" //as.pdf :p96
//.rodata
.global kalman //as.pdf : p254
/**
Comments for code readability
*/
//Your assembly code
kalman: //label
//PUSH current state to stack
//setup registers
//your function
//POP stack
// R0 is address of struct and R1 is the address of measurement. R2 is length.

VLDR S1, [R0] //S1 is q
VLDR S2, [R0,#4] //S2 is r
VLDR S3, [R0, #8] //S3 is x
VLDR S4, [R0, #12] //S4 is p
VLDR S5, [R0, #16] //S5 is k
//Assume S6 is measurement
update:
VADD S4, S4, S1 // p
VADD S7, S4, S2  // S7 is p+r
VDIV S5, S4, S7
VSUB S7, S6, S3 // S7 is now measurement - x
VMLA S3, S5, S7
VMLS S4, S4, S5


BX LR



