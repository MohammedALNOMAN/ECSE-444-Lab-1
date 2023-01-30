
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
VLDM.F32 R0, {S1-S5} //S1 is q, S2 is r, S3 is x, S4 is p, and S5 is k
//Assume S0 is measurement

VADD.F32 S4, S4, S1
VADD.F32 S7, S4, S2  // S7 is p+r
VDIV.F32 S5, S4, S7
VSUB.F32 S7, S0, S3 // S7 is now measurement - x
VMLA.F32 S3, S5, S7
VMLS.F32 S4, S4, S5

VSTM.F32 R0, {S1-S5} //S1 is q, S2 is r, S3 is x, S4 is p, and S5 is k
BX LR



