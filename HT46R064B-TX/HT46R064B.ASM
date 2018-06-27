#pragma debug scope 1 1
#line 33 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
_c_cmt_init .SECTION INPAGE PRIVATE 'CODE'
PUBLIC l_c_cmt_init
l_c_cmt_init:
ADDM A,[06h]
_c_cmt_init:
RET A,64
RET A,102
RET A,236
RET A,28
RET A,240
RET A,128
RET A,20
RET A,8
RET A,145
RET A,2
RET A,2
RET A,208
RET A,174
RET A,224
RET A,53
RET A,0
RET A,0
RET A,244
RET A,16
RET A,226
RET A,66
RET A,32
RET A,0
RET A,129
RET A,66
RET A,113
RET A,206
RET A,28
RET A,66
RET A,91
RET A,28
RET A,28
RET A,50
RET A,24
RET A,0
RET A,153
RET A,193
RET A,155
RET A,6
RET A,10
RET A,159
RET A,57
RET A,41
RET A,41
RET A,192
RET A,81
RET A,42
RET A,83
RET A,0
RET A,0
RET A,180
RET A,0
RET A,0
RET A,1
RET A,0
RET A,0
RET A,18
RET A,8
RET A,0
RET A,170
RET A,2
RET A,0
RET A,0
RET A,0
RET A,0
RET A,0
RET A,0
RET A,212
RET A,45
RET A,0
RET A,3
RET A,0
RET A,0
RET A,0
RET A,0
RET A,0
RET A,0
RET A,0
RET A,0
RET A,96
RET A,255
RET A,0
RET A,0
RET A,31
RET A,16
RET A,80
RET A,38
RET A,3
RET A,0
RET A,66
RET A,176
RET A,0
RET A,55
RET A,10
RET A,63
RET A,127
#line 53 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
__acsr EQU [023H]
__adcr EQU [022H]
__adrh EQU [021H]
__adrl EQU [020H]
__pwm0 EQU [01fH]
__intc1 EQU [01eH]
__ctrl1 EQU [01bH]
__ctrl0 EQU [01aH]
__pcpu EQU [019H]
__pcc EQU [018H]
__pc EQU [017H]
__pbpu EQU [016H]
__pbc EQU [015H]
__pb EQU [014H]
__pawk EQU [013H]
__papu EQU [012H]
__pac EQU [011H]
__pa EQU [010H]
__tmr0c EQU [0dH]
__tmr0 EQU [0cH]
__intc0 EQU [0bH]
__status EQU [0aH]
__wdts EQU [09H]
__tblh EQU [08H]
__tblp EQU [07H]
__pcl EQU [06H]
__acc EQU [05H]
__mp1 EQU [03H]
__mp0 EQU [01H]
__iar1 EQU [02H]
__iar0 EQU [00H]
@FIFO_WRITE .SECTION 'CODE'
PUBLIC _FIFO_WRITE
_FIFO_WRITE PROC
#pragma debug scope 2 1
#line 52 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
PUBLIC FIFO_WRITE0
LOCAL FIFO_WRITE0 DB ? ; len
#pragma debug variable 8 2 FIFO_WRITE0 len
#pragma debug scope 2 1 ; 55 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
#line 55 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SET [010H].1
#line 56 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR [010H].0
#line 57 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR [010H].3
#line 58 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR [011H].2
#line 59 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR CR3
#line 59 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
JMP L5
L2:
#pragma debug scope 3 2 ; 61 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
#line 61 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR [010H].0
#line 62 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,_a_tx
MOV CR2,A
#line 63 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR CR1
#line 63 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
JMP L9
L6:
#pragma debug scope 4 3 ; 65 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
#line 65 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR [010H].3
#line 66 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SNZ CR2.7
JMP L10
#line 66 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SET [010H].2
#line 66 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
JMP L11
L10:
#line 67 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR [010H].2
L11:
#line 68 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SET [010H].3
#line 69 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR [0AH].0
RLC CR2
L7:
#line 63 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
#pragma debug scope 3 2 ; 63 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
INC CR1
L9:
#line 63 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,CR1
SUB A,08h
SNZ [0ah].0
JMP L6
#line 71 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR [010H].3
#line 72 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,01H
CALL __DELAYX2
#line 73 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SET [010H].0
#line 74 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,01H
CALL __DELAYX2
L3:
#line 59 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
#pragma debug scope 2 1 ; 59 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
INC CR3
L5:
#line 59 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,CR3
SUB A,FIFO_WRITE0
SNZ [0ah].0
JMP L2
#line 76 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR [010H].3
#line 77 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SET [010H].0
L1:
RET
#line 54 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
LOCAL CR1 DB ? ; i
#pragma debug variable 8 2 CR1 i
#line 54 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
LOCAL CR2 DB ? ; data
#pragma debug variable 8 2 CR2 data
#line 54 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
LOCAL CR3 DB ? ; j
#pragma debug variable 8 2 CR3 j
_FIFO_WRITE ENDP
#line 81 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
@SPI_WRITE .SECTION 'CODE'
PUBLIC _SPI_WRITE
_SPI_WRITE PROC
#pragma debug scope 5 1
#line 80 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
PUBLIC SPI_WRITE0
LOCAL SPI_WRITE0 DB ? ; address
#pragma debug variable 8 5 SPI_WRITE0 address
#line 80 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
PUBLIC SPI_WRITE1
LOCAL SPI_WRITE1 DB ? ; data
#pragma debug variable 8 5 SPI_WRITE1 data
#pragma debug scope 5 1 ; 83 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
#line 83 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR SPI_WRITE0.7
#line 84 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SET [010H].0
#line 85 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR [010H].1
#line 86 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR [011H].2
#line 87 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR [010H].3
#line 88 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR CR4
#line 88 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
JMP L16
L13:
#pragma debug scope 6 5 ; 90 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
#line 90 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR [010H].3
#line 91 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SNZ SPI_WRITE0.7
JMP L17
#line 91 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SET [010H].2
#line 91 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
JMP L18
L17:
#line 92 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR [010H].2
L18:
#line 93 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SET [010H].3
#line 94 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR [0AH].0
RLC SPI_WRITE0
L14:
#line 88 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
#pragma debug scope 5 1 ; 88 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
INC CR4
L16:
#line 88 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,CR4
SUB A,08h
SNZ [0ah].0
JMP L13
#line 96 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR CR4
#line 96 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
JMP L22
L19:
#pragma debug scope 7 5 ; 98 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
#line 98 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR [010H].3
#line 99 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SNZ SPI_WRITE1.7
JMP L23
#line 99 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SET [010H].2
#line 99 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
JMP L24
L23:
#line 100 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR [010H].2
L24:
#line 101 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR [0AH].0
RLC SPI_WRITE1
#line 102 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SET [010H].3
L20:
#line 96 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
#pragma debug scope 5 1 ; 96 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
INC CR4
L22:
#line 96 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,CR4
SUB A,08h
SNZ [0ah].0
JMP L19
#line 104 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR [010H].3
#line 105 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CALL __DELAY1
#line 106 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR [010H].2
#line 107 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SET [010H].1
L12:
RET
#line 82 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
LOCAL CR4 DB ? ; i
#pragma debug variable 8 5 CR4 i
_SPI_WRITE ENDP
#line 111 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
@SPI_READ .SECTION 'CODE'
PUBLIC _SPI_READ
_SPI_READ PROC
#pragma debug scope 8 1
#line 110 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
PUBLIC SPI_READ0
LOCAL SPI_READ0 DB ? ; address
#pragma debug variable 8 8 SPI_READ0 address
#pragma debug scope 8 1 ; 113 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
#line 113 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SET [010H].0
#line 114 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR [010H].1
#line 115 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR [010H].3
#line 116 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR [011H].2
#line 117 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SET SPI_READ0.7
#line 118 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR CR5
#line 118 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
JMP L29
L26:
#pragma debug scope 9 8 ; 120 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
#line 120 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR [010H].3
#line 121 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SNZ SPI_READ0.7
JMP L30
#line 121 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SET [010H].2
#line 121 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
JMP L31
L30:
#line 122 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR [010H].2
L31:
#line 123 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
NOP
#line 124 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SET [010H].3
#line 125 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR [0AH].0
RLC SPI_READ0
L27:
#line 118 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
#pragma debug scope 8 1 ; 118 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
INC CR5
L29:
#line 118 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,CR5
SUB A,08h
SNZ [0ah].0
JMP L26
#line 127 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SET [011H].2
#line 128 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR [010H].3
#line 129 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR CR6
#line 130 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR CR5
#line 130 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
JMP L35
L32:
#pragma debug scope 10 8 ; 132 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
#line 132 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR [0AH].0
RLC CR6
#line 133 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SET [010H].3
#line 134 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SNZ [010H].2
JMP L36
#line 134 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SET CR6.0
L36:
#line 135 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
NOP
#line 136 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR [010H].3
L33:
#line 130 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
#pragma debug scope 8 1 ; 130 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
INC CR5
L35:
#line 130 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,CR5
SUB A,08h
SNZ [0ah].0
JMP L32
#line 138 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR [010H].3
#line 139 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR [011H].2
#line 140 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR [010H].2
#line 141 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SET [010H].1
#line 142 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,CR6
L25:
RET
#line 112 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
LOCAL CR5 DB ? ; i
#pragma debug variable 8 8 CR5 i
#line 112 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
LOCAL CR6 DB ? ; data
#pragma debug variable 8 8 CR6 data
_SPI_READ ENDP
#line 146 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
@CMT_init .SECTION 'CODE'
PUBLIC _CMT_init
_CMT_init PROC
#pragma debug scope 11 1 ; 148 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
#line 148 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,07fh
MOV SPI_WRITE0,A
MOV A,0ffh
MOV SPI_WRITE1,A
CALL _SPI_WRITE
#line 149 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,026H
MOV RH,A
MOV A,0fbH
CALL __DELAYY3
#line 150 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,060h
MOV SPI_WRITE0,A
MOV A,02h
MOV SPI_WRITE1,A
CALL _SPI_WRITE
#line 152 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,061h
MOV SPI_READ0,A
CALL _SPI_READ
MOV CR7,A
#line 153 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SET CR7.4
#line 154 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,0dfh
MOV b0_4,A
MOV A,b0_4
ANDM A,CR7
#line 155 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,061h
MOV SPI_WRITE0,A
MOV A,CR7
MOV SPI_WRITE1,A
CALL _SPI_WRITE
#line 157 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,062h
MOV SPI_READ0,A
CALL _SPI_READ
MOV CR7,A
#line 158 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SET CR7.5
#line 159 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,062h
MOV SPI_WRITE0,A
MOV A,CR7
MOV SPI_WRITE1,A
CALL _SPI_WRITE
#line 161 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,0dh
MOV SPI_READ0,A
CALL _SPI_READ
MOV CR7,A
#line 162 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,07fh
ANDM A,CR7
#line 163 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,0bfh
MOV b0_4,A
MOV A,b0_4
ANDM A,CR7
#line 164 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,0dfh
MOV b0_4,A
MOV A,b0_4
ANDM A,CR7
#line 165 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,0dh
MOV SPI_WRITE0,A
MOV A,CR7
MOV SPI_WRITE1,A
CALL _SPI_WRITE
#line 167 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,06ah
MOV SPI_WRITE0,A
MOV A,0ffh
MOV SPI_WRITE1,A
CALL _SPI_WRITE
#line 168 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,06bh
MOV SPI_WRITE0,A
MOV A,0ffh
MOV SPI_WRITE1,A
CALL _SPI_WRITE
#line 170 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR CR7
#line 170 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
JMP L42
L39:
#pragma debug scope 12 11 ; 172 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
#line 172 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,CR7
MOV SPI_WRITE0,A
MOV A,0
ADD A,CR7
CALL l_c_cmt_init
MOV b0_4,A
MOV A,b0_4
MOV SPI_WRITE1,A
CALL _SPI_WRITE
#line 173 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR WDT
L40:
#line 170 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
#pragma debug scope 11 1 ; 170 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
INC CR7
L42:
#line 170 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,CR7
SUB A,060h
SNZ [0ah].0
JMP L39
#line 180 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,065h
MOV SPI_WRITE0,A
MOV A,021h
MOV SPI_WRITE1,A
CALL _SPI_WRITE
#line 181 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,010h
MOV CR7,A
#line 182 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SET CR7.5
#line 183 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,066h
MOV SPI_WRITE0,A
MOV A,CR7
MOV SPI_WRITE1,A
CALL _SPI_WRITE
#line 187 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,0ah
MOV CR7,A
#line 188 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,067h
MOV SPI_WRITE0,A
MOV A,CR7
MOV SPI_WRITE1,A
CALL _SPI_WRITE
#line 189 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,068h
MOV SPI_WRITE0,A
MOV A,020h
MOV SPI_WRITE1,A
CALL _SPI_WRITE
#line 190 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,060h
MOV SPI_WRITE0,A
MOV A,010h
MOV SPI_WRITE1,A
CALL _SPI_WRITE
#line 191 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,03aH
MOV RH,A
MOV A,079H
CALL __DELAYY3
L38:
RET
LOCAL b0_4 DB ?
#line 147 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
LOCAL CR7 DB ? ; tmp
#pragma debug variable 8 11 CR7 tmp
_CMT_init ENDP
#line 202 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
@CMT_TX .SECTION 'CODE'
PUBLIC _CMT_TX
_CMT_TX PROC
#pragma debug scope 13 1 ; 204 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
#line 204 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SZ _f_k2
JMP L55
JMP L44
L55:
#pragma debug scope 14 13 ; 206 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
#line 206 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR _f_k2
#line 207 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SZ _f_tx
JMP L46
#pragma debug scope 15 14 ; 209 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
#line 209 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,01h
MOV _f_tx,A
#line 210 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,06dh
MOV SPI_READ0,A
CALL _SPI_READ
MOV CR8,A
#line 211 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,060h
MOV SPI_WRITE0,A
MOV A,02h
MOV SPI_WRITE1,A
CALL _SPI_WRITE
#line 212 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,06dh
MOV SPI_READ0,A
CALL _SPI_READ
MOV CR8,A
#line 214 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,06ah
MOV SPI_WRITE0,A
MOV A,0ffh
MOV SPI_WRITE1,A
CALL _SPI_WRITE
#line 215 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,06bh
MOV SPI_WRITE0,A
MOV A,0ffh
MOV SPI_WRITE1,A
CALL _SPI_WRITE
#line 216 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,06dh
MOV SPI_READ0,A
CALL _SPI_READ
MOV CR8,A
#line 218 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,069h
MOV SPI_READ0,A
CALL _SPI_READ
MOV CR8,A
#line 219 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SET CR8.0
#line 220 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SET CR8.2
#line 221 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,069h
MOV SPI_WRITE0,A
MOV A,CR8
MOV SPI_WRITE1,A
CALL _SPI_WRITE
#line 223 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,06ch
MOV SPI_WRITE0,A
MOV A,01h
MOV SPI_WRITE1,A
CALL _SPI_WRITE
#line 224 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,020h
MOV FIFO_WRITE0,A
CALL _FIFO_WRITE
#line 225 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,06dh
MOV SPI_READ0,A
CALL _SPI_READ
MOV CR8,A
#line 227 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,060h
MOV SPI_WRITE0,A
MOV A,040h
MOV SPI_WRITE1,A
CALL _SPI_WRITE
#line 227 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
JMP L49
L48:
#pragma debug scope 16 15 ; 230 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
#line 230 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,060h
MOV SPI_READ0,A
CALL _SPI_READ
MOV CR8,A
#line 231 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,065h
MOV SPI_READ0,A
CALL _SPI_READ
MOV CR8,A
#line 232 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,06dh
MOV SPI_READ0,A
CALL _SPI_READ
MOV CR8,A
#line 233 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,068h
MOV SPI_READ0,A
CALL _SPI_READ
MOV CR8,A
#line 234 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,066h
MOV SPI_READ0,A
CALL _SPI_READ
MOV CR8,A
#line 235 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,067h
MOV SPI_READ0,A
CALL _SPI_READ
MOV CR8,A
#line 236 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
NOP
#line 237 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR WDT
#line 238 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,06ah
MOV SPI_WRITE0,A
MOV A,04h
MOV SPI_WRITE1,A
CALL _SPI_WRITE
#line 239 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,060h
MOV SPI_WRITE0,A
MOV A,010h
MOV SPI_WRITE1,A
CALL _SPI_WRITE
L49:
#line 228 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
#pragma debug scope 15 14 ; 228 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
JMP L48
L46:
L44:
#line 243 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
#pragma debug scope 13 1 ; 243 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SZ _f_tx
JMP L56
JMP L51
L56:
#pragma debug scope 17 13 ; 245 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
#line 245 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SZ [014H].1
JMP L53
#pragma debug scope 18 17 ; 247 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
#line 247 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR _f_tx
#line 248 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,06ah
MOV SPI_WRITE0,A
MOV A,04h
MOV SPI_WRITE1,A
CALL _SPI_WRITE
#line 249 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,060h
MOV SPI_WRITE0,A
MOV A,010h
MOV SPI_WRITE1,A
CALL _SPI_WRITE
L53:
L51:
L43:
RET
#line 203 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
LOCAL CR8 DB ? ; tmp
#pragma debug variable 8 13 CR8 tmp
_CMT_TX ENDP
#line 255 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
@KEY .SECTION 'CODE'
PUBLIC _KEY
_KEY PROC
#pragma debug scope 19 1 ; 256 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
#line 256 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SET [015H].2
#line 257 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SET [016H].2
#line 258 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SZ [014H].2
JMP L58
#pragma debug scope 20 19 ; 260 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
#line 260 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR _a_k2_high
#line 261 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SZ _f_k2_buf
JMP L59
#pragma debug scope 21 20 ; 263 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
#line 263 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
INC _a_k2_low
#line 264 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,_a_k2_low
SUB A,05h
SNZ [0ah].0
JMP L59
#pragma debug scope 22 21 ; 266 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
#line 266 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,01h
MOV _f_k2,A
#line 267 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,01h
MOV _f_k2_buf,A
#line 270 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
#pragma debug scope 19 1 ; 270 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
JMP L59
L58:
#pragma debug scope 23 19 ; 273 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
#line 273 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR _a_k2_low
#line 274 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SZ _f_k2_buf
JMP L68
JMP L64
L68:
#pragma debug scope 24 23 ; 276 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
#line 276 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
INC _a_k2_high
#line 277 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,_a_k2_high
SUB A,05h
SNZ [0ah].0
JMP L66
#pragma debug scope 25 24 ; 279 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
#line 279 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR _f_k2_buf
L66:
L64:
L59:
L57:
RET
_KEY ENDP
#line 286 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
@initail .SECTION 'CODE'
PUBLIC _initail
_initail PROC
#pragma debug scope 26 1 ; 287 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
#line 287 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,040h
MOV __mp0,A
#line 287 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
JMP L73
L70:
#line 287 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR __iar0
L71:
#line 287 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
INC __mp0
L73:
#line 287 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,__mp0
SUB A,0ffh
SNZ [0AH].0
JMP L70
#line 288 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR __iar0
#line 289 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR [011H].0
#line 290 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR [011H].1
#line 291 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR [011H].3
#line 292 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR [011H].2
#line 293 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SET [010H].0
#line 294 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SET [010H].1
#line 295 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR [010H].3
#line 296 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR [010H].2
#line 297 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SET [012H].2
#line 299 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,055h
MOV _a_tx,A
#line 300 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,0aah
MOV _a_tx[1],A
#line 301 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
MOV A,01h
MOV _f_k2,A
#line 302 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SET [015H].1
#line 303 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
SET [016H].1
#line 305 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CALL _CMT_init
L69:
RET
_initail ENDP
#line 309 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
@MAIN .SECTION 'CODE'
_main PROC
begin:
#pragma debug scope 27 1 ; 311 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
#line 311 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CALL _initail
#line 311 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
JMP L77
L76:
#pragma debug scope 28 27 ; 314 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
#line 314 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CLR WDT
#line 319 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
CALL _CMT_TX
#line 320 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
NOP
L77:
#line 312 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
#pragma debug scope 27 1 ; 312 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
JMP L76
L75:
#line 322 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
JMP $
_main ENDP
PUBLIC _a_tx
@a_tx .SECTION 'DATA'
#line 30 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
_a_tx DB 2 DUP (?) ; a_tx
#pragma debug array 42 8 2 1 _a_tx a_tx
PUBLIC _a_data
@a_data .SECTION 'DATA'
#line 30 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
_a_data DB ? ; a_data
#pragma debug variable 8 1 _a_data a_data
PUBLIC _f_10ms
@f_10ms .SECTION 'DATA'
#line 29 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
_f_10ms DB ? ; f_10ms
#pragma debug variable 8 1 _f_10ms f_10ms
PUBLIC _f_k2_buf
@f_k2_buf .SECTION 'DATA'
#line 29 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
_f_k2_buf DB ? ; f_k2_buf
#pragma debug variable 8 1 _f_k2_buf f_k2_buf
PUBLIC _f_k1_buf
@f_k1_buf .SECTION 'DATA'
#line 29 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
_f_k1_buf DB ? ; f_k1_buf
#pragma debug variable 8 1 _f_k1_buf f_k1_buf
PUBLIC _f_tx
@f_tx .SECTION 'DATA'
#line 29 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
_f_tx DB ? ; f_tx
#pragma debug variable 8 1 _f_tx f_tx
PUBLIC _f_k2
@f_k2 .SECTION 'DATA'
#line 29 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
_f_k2 DB ? ; f_k2
#pragma debug variable 8 1 _f_k2 f_k2
PUBLIC _f_k1
@f_k1 .SECTION 'DATA'
#line 29 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
_f_k1 DB ? ; f_k1
#pragma debug variable 8 1 _f_k1 f_k1
PUBLIC _a_k5_low
@a_k5_low .SECTION 'DATA'
#line 28 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
_a_k5_low DB ? ; a_k5_low
#pragma debug variable 8 1 _a_k5_low a_k5_low
PUBLIC _a_k5_high
@a_k5_high .SECTION 'DATA'
#line 28 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
_a_k5_high DB ? ; a_k5_high
#pragma debug variable 8 1 _a_k5_high a_k5_high
PUBLIC _a_k4_low
@a_k4_low .SECTION 'DATA'
#line 28 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
_a_k4_low DB ? ; a_k4_low
#pragma debug variable 8 1 _a_k4_low a_k4_low
PUBLIC _a_k4_high
@a_k4_high .SECTION 'DATA'
#line 28 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
_a_k4_high DB ? ; a_k4_high
#pragma debug variable 8 1 _a_k4_high a_k4_high
PUBLIC _a_k3_low
@a_k3_low .SECTION 'DATA'
#line 28 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
_a_k3_low DB ? ; a_k3_low
#pragma debug variable 8 1 _a_k3_low a_k3_low
PUBLIC _a_k3_high
@a_k3_high .SECTION 'DATA'
#line 28 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
_a_k3_high DB ? ; a_k3_high
#pragma debug variable 8 1 _a_k3_high a_k3_high
PUBLIC _a_k2_low
@a_k2_low .SECTION 'DATA'
#line 28 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
_a_k2_low DB ? ; a_k2_low
#pragma debug variable 8 1 _a_k2_low a_k2_low
PUBLIC _a_k2_high
@a_k2_high .SECTION 'DATA'
#line 28 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
_a_k2_high DB ? ; a_k2_high
#pragma debug variable 8 1 _a_k2_high a_k2_high
PUBLIC _a_k1_low
@a_k1_low .SECTION 'DATA'
#line 28 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
_a_k1_low DB ? ; a_k1_low
#pragma debug variable 8 1 _a_k1_low a_k1_low
PUBLIC _a_k1_high
@a_k1_high .SECTION 'DATA'
#line 28 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
_a_k1_high DB ? ; a_k1_high
#pragma debug variable 8 1 _a_k1_high a_k1_high
PUBLIC _a_10ms
@a_10ms .SECTION 'DATA'
#line 27 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
_a_10ms DB ? ; a_10ms
#pragma debug variable 8 1 _a_10ms a_10ms
PUBLIC _a_count
@a_count .SECTION 'DATA'
#line 30 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
_a_count DB ? ; a_count
#pragma debug variable 8 1 _a_count a_count
PUBLIC _a_100ms
@a_100ms .SECTION 'DATA'
#line 27 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
_a_100ms DB ? ; a_100ms
#pragma debug variable 8 1 _a_100ms a_100ms
PUBLIC _lcd_data
@lcd_data .SECTION 'DATA'
#line 27 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
_lcd_data DB 2 DUP (?) ; lcd_data
#pragma debug array 43 8 2 1 _lcd_data lcd_data
PUBLIC _a_lcd_count
@a_lcd_count .SECTION 'DATA'
#line 27 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
_a_lcd_count DB ? ; a_lcd_count
#pragma debug variable 8 1 _a_lcd_count a_lcd_count
rambank 0 @a_rx
@a_rx .SECTION AT 040H 'DATA'
PUBLIC _a_rx
#line 26 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
_a_rx DB 16 DUP (?) ; a_rx
#pragma debug array 44 8 16 1 _a_rx a_rx
#line 308 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
STARTSEC .SECTION  COM_L 'CODE'
STARTUP:
STARTSEC .SECTION  COM_E 'CODE'
RET
EXTERN RH:BYTE
EXTERN __DELAY0:NEAR
EXTERN __DELAY1:NEAR
EXTERN __DELAY2:NEAR
EXTERN __DELAY3:NEAR
EXTERN __DELAYX0:NEAR
EXTERN __DELAYX1:NEAR
EXTERN __DELAYX2:NEAR
EXTERN __DELAYX3:NEAR
EXTERN __DELAYX4:NEAR
EXTERN __DELAYX5:NEAR
EXTERN __DELAYX6:NEAR
EXTERN __DELAYY0:NEAR
EXTERN __DELAYY1:NEAR
EXTERN __DELAYY2:NEAR
EXTERN __DELAYY3:NEAR
EXTERN __DELAYY4:NEAR
EXTERN __DELAYY5:NEAR
#line 308 "D:\Project\pcollar640\HT46R064B-TX\HT46R064B.c"
@CODE .SECTION AT 0 'CODE'
CALL STARTUP
JMP begin
@DUMMY PROC
CALL _main
@DUMMY ENDP
