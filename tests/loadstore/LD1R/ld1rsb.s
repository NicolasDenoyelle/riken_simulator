	.ident	"$Options: Fujitsu C/C++ Compiler Version 1.2.0 P-id: Alpha-20161222 (Dec 20 2016 18:37:40) --preinclude /opt/FJTComp3/bin/../lib/c99.pre --gcc --c99 -D__FUJITSU -Dunix -D__aarch64__ -D__unix -D__fcc_version__=0x700 -D__fcc_version=700 -D__USER_LABEL_PREFIX__= -D__OPTIMIZE__ -D__ARM_ARCH=8 -D__FP_FAST_FMA -D__ELF__ -D__unix__ -D__linux__ -D__linux -Asystem(unix) -Dlinux -D__LIBC_6B -D__LP64__ -D_LP64 --K=noocl --zmode=64 --sys_include=/opt/FJTComp3/bin/../include --sys_include=/dummy/usr/include --K=opt ld1rsb.c -- -ncmdname=fccpx -Nnoline -zobe=no-static-clump -O3 -x- -Komitfp,mfunc,rdconv,eval,fp_relaxed,ns,fast_matmul,fp_contract,ilfunc,prefetch_conditional,loop_nofission,nounroll ld1rsb.s $"
	.file	"ld1rsb.c"
	.ident	"$Compiler: Fujitsu C/C++ Compiler Version 1.2.0 P-id: Alpha-20161222 (Dec 20 2016 18:37:40) ld1rsb.c main $"
	.text
	.align	2
	.global	main
	.type	main, %function
main:
/*    ??? */	stp	x19, x20, [sp, -80]!	//  (*)
/*    ??? */	stp	x21, x22, [sp, 16]	//  (*)
/*    ??? */	str	x30, [sp, 32]	//  (*)
/*     13 */	ptrue	p0.s, VL8
/*     13 */	index	z0.s, 1, 1
/*     15 */	add	x22, sp, 56
/*     14 */	add	x0, sp, 72
/*     15 */	dup	z1.h, 0
/*     15 */	st1h	z1.s, p0, [x22, 0, mul vl]	//  "y"
/*     14 */	add	z0.s, z0.s, 0
/*     14 */	st1b	z0.s, p0, [x0, 0, mul vl]	//  "x"
/*     18 */	ptrue	p0.h, VL8
/*     19 */	ld1rsb	z0.h, p0/z, [x0, 2]
/*     19 */	st1h	z0.h, p0, [x22, 0, mul vl]	//  "y"
/*     22 */	mov	w20, 8
/*     23 */	mov	x21, 0
/*     23 */	adrp	x0, .LR.2
/*     23 */	add	x19, x0, :lo12:.LR.2
.L85:					// :entr:term
/*     23 */	ldrh	w1, [x22, x21, lsl #1]	//  "y"
/*     23 */	mov	x0, x19
/*     23 */	bl	printf
/*     24 */	add	x21, x21, 1
/*     24 */	subs	w20, w20, 1
/*     24 */	bne	.L85
/*     25 */	adrp	x0, .LR.1
/*     25 */	add	x0, x0, :lo12:.LR.1
/*     25 */	bl	printf
/*     27 */	mov	w0, 0
/*    ??? */	ldp	x21, x22, [sp, 16]	//  (*)
/*    ??? */	ldr	x30, [sp, 32]	//  (*)
/*    ??? */	ldp	x19, x20, [sp], 80	//  (*)
/*     28 */	ret	
	.size	main, .-main
	.section	.rodata
	.align	3
.LR.1:
	.ascii "\012\000"
	.type	.LR.1, %object
	.size	.LR.1,.-.LR.1
	.section	.rodata
	.align	3
.LR.2:
	.ascii "%u\040\000"
	.type	.LR.2, %object
	.size	.LR.2,.-.LR.2
	.section	.note.GNU-stack,"",%progbits
