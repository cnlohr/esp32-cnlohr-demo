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
