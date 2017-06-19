#define XCHAL_NMILEVEL       7/* NMI "level" (for use with */


.global asmtest
.align 4

asmtest:
	entry   a1, 32

//	addi a1, a1, -68
	s32i.n a0, a1, 0   // Working reg
	s32i.n a2, a1, 4   // Running byte
	s32i.n a3, a1, 8   // Running CRC
	s32i.n a4, a1, 12  // Anding mask

	movi a0,160000
	movi a2, GPIO    //Will take 10 cycles to read (at 240 MHz) 5 at 80. and 7 at 160, 1 to write.
//	movi a2, mydata  //Will take 1 cycle

//	l32i a3, a2, 0
keep_going:
	l32i a3, a2, 0
//	s32i a3, a2, 0
	addi a0, a0, -1
	bnez a0, keep_going



	l32i.n a0, a1, 0
	l32i.n a2, a1, 4
	l32i.n a3, a1, 8
	l32i.n a4, a1, 12
//	addi a1, a1, 68
	retw


.align 4
.global test_fast_gpio
test_fast_gpio:
	entry   a1, 32
	addi a1, a1, -68

	s32i.n a0, a1, 0
	s32i.n a3, a1, 4
	s32i.n a4, a1, 8
	movi a4, 10000

tg:
	movi a2, GPIO
	movi a3, 1<<17
	s32i a3, a2, 8   //W1TS
	s32i a3, a2, 12  //W1TC
	addi a4, a4, -1
	bnez a4, tg

	l32i.n a4, a1, 8
	l32i.n a3, a1, 4
	movi a2, GPIO
	l32i.n a0, a1, 0
	addi a1, a1, 68
	retw
	//j test_fast_gpio






//My terrible ISR WORKS!
	.global _my_xt_nmi
    .section .iram1,"ax"
    .type       _my_xt_nmi,@function
    .align      4

_my_xt_nmi:

	addi a1, a1, -68
	s32i a2, a1, 4
	s32i a3, a1, 8
	s32i a4, a1, 12


	movi a2, GPIO
	movi a3, 1<<16
	movi a4, 100
_xt_loop:
	s32i a3, a2, 8   //W1TS
	s32i a3, a2, 12  //W1TC
	addi a4, a4, -1
	bnez a4, _xt_loop

	movi a3, 1
	s32i a3, a2, 0x4c //Status w1tc

	l32i a4, a1, 12
	l32i a3, a1, 8
	l32i a2, a1, 4
	addi a1, a1, 68
    rsr     a0, EXCSAVE7    /* restore a0 */
    rfi     XCHAL_NMILEVEL

