#pragma debug scope 1 1
#line 20 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
_c_cmt_init .SECTION INPAGE PRIVATE 'CODE'
PUBLIC l_c_cmt_init
l_c_cmt_init:
ADDM A,[06h]
_c_cmt_init:
RET A,2
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
RET A,13
RET A,174
RET A,224
RET A,112
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
RET A,31
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
RET A,1
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
RET A,1
RET A,1
RET A,63
RET A,127
#line 31 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
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
@FIFO_READ .SECTION 'CODE'
PUBLIC _FIFO_READ
_FIFO_READ PROC
#pragma debug scope 2 1
#line 30 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
PUBLIC FIFO_READ0
LOCAL FIFO_READ0 DB ? ; len
#pragma debug variable 8 2 FIFO_READ0 len
#pragma debug scope 2 1 ; 33 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
#line 33 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
SET [010H].1
#line 34 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [010H].0
#line 35 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [010H].3
#line 36 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
SET [011H].2
#line 37 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR CR3
#line 37 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
JMP L5
L2:
#pragma debug scope 3 2 ; 39 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
#line 39 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [010H].0
#line 40 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR CR2
#line 41 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR CR1
#line 41 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
JMP L9
L6:
#pragma debug scope 4 3 ; 43 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
#line 43 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [0AH].0
RLC CR2
#line 44 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
SET [010H].3
#line 45 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
SNZ [010H].2
JMP L10
#line 45 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
SET CR2.0
L10:
#line 46 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
NOP
#line 47 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [010H].3
L7:
#line 41 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
#pragma debug scope 3 2 ; 41 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
INC CR1
L9:
#line 41 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,CR1
SUB A,08h
SNZ [0ah].0
JMP L6
#line 49 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,CR3
ADD A,OFFSET _a_rev
MOV b0_1,A
MOV A,b0_1
MOV [01H],A
MOV A,CR2
MOV [00H],A
#line 50 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
SET [010H].0
#line 51 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,01H
CALL __DELAYX2
L3:
#line 37 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
#pragma debug scope 2 1 ; 37 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
INC CR3
L5:
#line 37 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,CR3
SUB A,FIFO_READ0
SNZ [0ah].0
JMP L2
#line 53 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [011H].2
#line 54 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [010H].3
#line 55 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
SET [010H].0
L1:
RET
LOCAL b0_1 DB ?
#line 32 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
LOCAL CR1 DB ? ; i
#pragma debug variable 8 2 CR1 i
#line 32 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
LOCAL CR2 DB ? ; data
#pragma debug variable 8 2 CR2 data
#line 32 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
LOCAL CR3 DB ? ; j
#pragma debug variable 8 2 CR3 j
_FIFO_READ ENDP
#line 59 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
@FIFO_WRITE .SECTION 'CODE'
PUBLIC _FIFO_WRITE
_FIFO_WRITE PROC
#pragma debug scope 5 1
#line 58 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
PUBLIC FIFO_WRITE0
LOCAL FIFO_WRITE0 DB ? ; len
#pragma debug variable 8 5 FIFO_WRITE0 len
#pragma debug scope 5 1 ; 61 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
#line 61 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
SET [010H].1
#line 62 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [010H].0
#line 63 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [010H].3
#line 64 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [011H].2
#line 65 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR CR6
#line 65 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
JMP L16
L13:
#pragma debug scope 6 5 ; 67 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
#line 67 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [010H].0
#line 68 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,OFFSET _a_tx
ADD A,CR6
MOV [01H],A
MOV A,[00H]
MOV b0_2,A
MOV A,b0_2
MOV CR5,A
#line 69 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR CR4
#line 69 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
JMP L20
L17:
#pragma debug scope 7 6 ; 71 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
#line 71 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [010H].3
#line 72 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
SNZ CR5.7
JMP L21
#line 72 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
SET [010H].2
#line 72 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
JMP L22
L21:
#line 73 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [010H].2
L22:
#line 74 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
SET [010H].3
#line 75 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [0AH].0
RLC CR5
L18:
#line 69 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
#pragma debug scope 6 5 ; 69 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
INC CR4
L20:
#line 69 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,CR4
SUB A,08h
SNZ [0ah].0
JMP L17
#line 77 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
SET [010H].0
#line 78 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,01H
CALL __DELAYX2
L14:
#line 65 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
#pragma debug scope 5 1 ; 65 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
INC CR6
L16:
#line 65 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,CR6
SUB A,FIFO_WRITE0
SNZ [0ah].0
JMP L13
#line 80 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [010H].3
#line 81 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
SET [010H].0
L12:
RET
LOCAL b0_2 DB ?
#line 60 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
LOCAL CR4 DB ? ; i
#pragma debug variable 8 5 CR4 i
#line 60 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
LOCAL CR5 DB ? ; data
#pragma debug variable 8 5 CR5 data
#line 60 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
LOCAL CR6 DB ? ; j
#pragma debug variable 8 5 CR6 j
_FIFO_WRITE ENDP
#line 85 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
@SPI_WRITE .SECTION 'CODE'
PUBLIC _SPI_WRITE
_SPI_WRITE PROC
#pragma debug scope 8 1
#line 84 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
PUBLIC SPI_WRITE0
LOCAL SPI_WRITE0 DB ? ; address
#pragma debug variable 8 8 SPI_WRITE0 address
#line 84 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
PUBLIC SPI_WRITE1
LOCAL SPI_WRITE1 DB ? ; data
#pragma debug variable 8 8 SPI_WRITE1 data
#pragma debug scope 8 1 ; 87 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
#line 87 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR SPI_WRITE0.7
#line 88 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
SET [010H].0
#line 89 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [010H].1
#line 90 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [011H].2
#line 91 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [010H].3
#line 92 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR CR7
#line 92 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
JMP L27
L24:
#pragma debug scope 9 8 ; 94 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
#line 94 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [010H].3
#line 95 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
SNZ SPI_WRITE0.7
JMP L28
#line 95 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
SET [010H].2
#line 95 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
JMP L29
L28:
#line 96 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [010H].2
L29:
#line 97 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
SET [010H].3
#line 98 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [0AH].0
RLC SPI_WRITE0
L25:
#line 92 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
#pragma debug scope 8 1 ; 92 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
INC CR7
L27:
#line 92 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,CR7
SUB A,08h
SNZ [0ah].0
JMP L24
#line 100 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR CR7
#line 100 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
JMP L33
L30:
#pragma debug scope 10 8 ; 102 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
#line 102 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [010H].3
#line 103 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
SNZ SPI_WRITE1.7
JMP L34
#line 103 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
SET [010H].2
#line 103 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
JMP L35
L34:
#line 104 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [010H].2
L35:
#line 105 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [0AH].0
RLC SPI_WRITE1
#line 106 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
SET [010H].3
L31:
#line 100 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
#pragma debug scope 8 1 ; 100 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
INC CR7
L33:
#line 100 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,CR7
SUB A,08h
SNZ [0ah].0
JMP L30
#line 108 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [010H].3
#line 109 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CALL __DELAY1
#line 110 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [010H].2
#line 111 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
SET [010H].1
L23:
RET
#line 86 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
LOCAL CR7 DB ? ; i
#pragma debug variable 8 8 CR7 i
_SPI_WRITE ENDP
#line 115 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
@SPI_READ .SECTION 'CODE'
PUBLIC _SPI_READ
_SPI_READ PROC
#pragma debug scope 11 1
#line 114 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
PUBLIC SPI_READ0
LOCAL SPI_READ0 DB ? ; address
#pragma debug variable 8 11 SPI_READ0 address
#pragma debug scope 11 1 ; 117 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
#line 117 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
SET [010H].0
#line 118 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [010H].1
#line 119 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [010H].3
#line 120 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [011H].2
#line 121 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
SET SPI_READ0.7
#line 122 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR CR8
#line 122 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
JMP L40
L37:
#pragma debug scope 12 11 ; 124 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
#line 124 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [010H].3
#line 125 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
SNZ SPI_READ0.7
JMP L41
#line 125 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
SET [010H].2
#line 125 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
JMP L42
L41:
#line 126 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [010H].2
L42:
#line 127 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
NOP
#line 128 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
SET [010H].3
#line 129 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [0AH].0
RLC SPI_READ0
L38:
#line 122 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
#pragma debug scope 11 1 ; 122 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
INC CR8
L40:
#line 122 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,CR8
SUB A,08h
SNZ [0ah].0
JMP L37
#line 131 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
SET [011H].2
#line 132 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [010H].3
#line 133 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR CR9
#line 134 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR CR8
#line 134 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
JMP L46
L43:
#pragma debug scope 13 11 ; 136 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
#line 136 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [0AH].0
RLC CR9
#line 137 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
SET [010H].3
#line 138 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
SNZ [010H].2
JMP L47
#line 138 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
SET CR9.0
L47:
#line 139 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
NOP
#line 140 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [010H].3
L44:
#line 134 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
#pragma debug scope 11 1 ; 134 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
INC CR8
L46:
#line 134 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,CR8
SUB A,08h
SNZ [0ah].0
JMP L43
#line 142 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [010H].3
#line 143 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [011H].2
#line 144 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [010H].2
#line 145 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
SET [010H].1
#line 146 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,CR9
L36:
RET
#line 116 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
LOCAL CR8 DB ? ; i
#pragma debug variable 8 11 CR8 i
#line 116 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
LOCAL CR9 DB ? ; data
#pragma debug variable 8 11 CR9 data
_SPI_READ ENDP
#line 150 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
@CMT_init .SECTION 'CODE'
PUBLIC _CMT_init
_CMT_init PROC
#pragma debug scope 14 1 ; 152 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
#line 152 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,07fh
MOV SPI_WRITE0,A
MOV A,0ffh
MOV SPI_WRITE1,A
CALL _SPI_WRITE
#line 153 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,026H
MOV RH,A
MOV A,0fbH
CALL __DELAYY3
#line 154 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,060h
MOV SPI_WRITE0,A
MOV A,02h
MOV SPI_WRITE1,A
CALL _SPI_WRITE
#line 156 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,061h
MOV SPI_READ0,A
CALL _SPI_READ
MOV CR10,A
#line 157 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
SET CR10.4
#line 158 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,0dfh
MOV b0_5,A
MOV A,b0_5
ANDM A,CR10
#line 159 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,061h
MOV SPI_WRITE0,A
MOV A,CR10
MOV SPI_WRITE1,A
CALL _SPI_WRITE
#line 161 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,062h
MOV SPI_READ0,A
CALL _SPI_READ
MOV CR10,A
#line 162 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
SET CR10.5
#line 163 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,062h
MOV SPI_WRITE0,A
MOV A,CR10
MOV SPI_WRITE1,A
CALL _SPI_WRITE
#line 165 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR CR10
#line 165 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
JMP L53
L50:
#pragma debug scope 15 14 ; 167 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
#line 167 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,CR10
MOV SPI_WRITE0,A
MOV A,0
ADD A,CR10
CALL l_c_cmt_init
MOV b0_5,A
MOV A,b0_5
MOV SPI_WRITE1,A
CALL _SPI_WRITE
#line 168 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR WDT
L51:
#line 165 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
#pragma debug scope 14 1 ; 165 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
INC CR10
L53:
#line 165 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,CR10
SUB A,060h
SNZ [0ah].0
JMP L50
#line 173 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,060h
MOV SPI_WRITE0,A
MOV A,010h
MOV SPI_WRITE1,A
CALL _SPI_WRITE
#line 175 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,065h
MOV SPI_WRITE0,A
MOV A,015h
MOV SPI_WRITE1,A
CALL _SPI_WRITE
#line 176 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,066h
MOV SPI_READ0,A
CALL _SPI_READ
MOV CR10,A
#line 177 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,039h
ORM A,CR10
#line 178 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,066h
MOV SPI_WRITE0,A
MOV A,CR10
MOV SPI_WRITE1,A
CALL _SPI_WRITE
#line 179 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,067h
MOV SPI_READ0,A
CALL _SPI_READ
MOV CR10,A
#line 180 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,CR10
MOV CR10,A
#line 181 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,066h
MOV SPI_WRITE0,A
MOV A,CR10
MOV SPI_WRITE1,A
CALL _SPI_WRITE
#line 182 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,068h
MOV SPI_WRITE0,A
MOV A,01h
MOV SPI_WRITE1,A
CALL _SPI_WRITE
#line 183 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,060h
MOV SPI_WRITE0,A
MOV A,010h
MOV SPI_WRITE1,A
CALL _SPI_WRITE
#line 185 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,048h
MOV SPI_READ0,A
CALL _SPI_READ
MOV CR11,A
#line 186 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,048h
MOV SPI_WRITE0,A
MOV A,0aah
MOV SPI_WRITE1,A
CALL _SPI_WRITE
#line 187 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,048h
MOV SPI_READ0,A
CALL _SPI_READ
MOV CR10,A
#line 188 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,048h
MOV SPI_WRITE0,A
MOV A,CR11
MOV SPI_WRITE1,A
CALL _SPI_WRITE
#line 189 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,CR10
XOR A,0aah
SZ [0AH].2
JMP L54
L56:
L57:
#line 189 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
JMP L56
L54:
#line 191 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,060h
MOV SPI_WRITE0,A
MOV A,02h
MOV SPI_WRITE1,A
CALL _SPI_WRITE
#line 192 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,06bh
MOV SPI_WRITE0,A
MOV A,01h
MOV SPI_WRITE1,A
CALL _SPI_WRITE
#line 194 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,069h
MOV SPI_READ0,A
CALL _SPI_READ
MOV CR10,A
#line 195 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR CR10.0
#line 196 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR CR10.2
#line 197 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,069h
MOV SPI_WRITE0,A
MOV A,CR10
MOV SPI_WRITE1,A
CALL _SPI_WRITE
#line 199 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,06ch
MOV SPI_WRITE0,A
MOV A,02h
MOV SPI_WRITE1,A
CALL _SPI_WRITE
#line 201 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,060h
MOV SPI_WRITE0,A
MOV A,08h
MOV SPI_WRITE1,A
CALL _SPI_WRITE
L49:
RET
LOCAL b0_5 DB ?
#line 151 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
LOCAL CR10 DB ? ; tmp
#pragma debug variable 8 14 CR10 tmp
#line 151 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
LOCAL CR11 DB ? ; back
#pragma debug variable 8 14 CR11 back
_CMT_init ENDP
#line 205 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
@CMT_RX .SECTION 'CODE'
PUBLIC _CMT_RX
_CMT_RX PROC
#pragma debug scope 16 1 ; 207 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
#line 207 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
SNZ [014H].0
JMP L60
#pragma debug scope 17 16 ; 209 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
#line 209 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,02h
MOV FIFO_READ0,A
CALL _FIFO_READ
#line 210 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,06bh
MOV SPI_WRITE0,A
MOV A,01h
MOV SPI_WRITE1,A
CALL _SPI_WRITE
#line 211 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,060h
MOV SPI_WRITE0,A
MOV A,010h
MOV SPI_WRITE1,A
CALL _SPI_WRITE
#line 212 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,_a_rev
XOR A,_a_rev[1]
SZ [05H]
JMP L62
#line 212 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR b0_6
SZ [014H].0
SET b0_6.0
MOV A,01h
XORM A,b0_6
MOV A,[014H]
CLR [05H].0
SZ b0_6
SET [05H].0
MOV [014H],A
L62:
L60:
L59:
RET
LOCAL b0_6 DB ?
_CMT_RX ENDP
#line 217 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
@initail .SECTION 'CODE'
PUBLIC _initail
_initail PROC
#pragma debug scope 18 1 ; 218 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
#line 218 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [011H].0
#line 219 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [011H].1
#line 220 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [011H].3
#line 221 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [011H].2
#line 222 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
SET [010H].0
#line 223 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
SET [010H].1
#line 224 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [010H].3
#line 225 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR [010H].2
#line 226 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
SET [012H].2
#line 228 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,055h
MOV _a_tx,A
#line 229 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,0aah
MOV _a_tx[1],A
L65:
RET
_initail ENDP
#line 233 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
@MAIN .SECTION 'CODE'
_main PROC
begin:
#pragma debug scope 19 1 ; 234 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
#line 234 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CALL _initail
#line 234 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
JMP L69
L68:
#pragma debug scope 20 19 ; 237 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
#line 237 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
CLR WDT
#line 238 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,02h
MOV FIFO_WRITE0,A
CALL _FIFO_WRITE
#line 239 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
MOV A,0f9H
CALL __DELAYX0
L69:
#line 235 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
#pragma debug scope 19 1 ; 235 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
JMP L68
L67:
#line 241 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
JMP $
_main ENDP
PUBLIC _a_tx
@a_tx .SECTION 'DATA'
#line 18 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
_a_tx DB 2 DUP (?) ; a_tx
#pragma debug array 42 8 2 1 _a_tx a_tx
PUBLIC _a_rev
@a_rev .SECTION 'DATA'
#line 18 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
_a_rev DB 2 DUP (?) ; a_rev
#pragma debug array 43 8 2 1 _a_rev a_rev
PUBLIC _a_count
@a_count .SECTION 'DATA'
#line 18 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
_a_count DB ? ; a_count
#pragma debug variable 8 1 _a_count a_count
PUBLIC _a_data
@a_data .SECTION 'DATA'
#line 18 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
_a_data DB ? ; a_data
#pragma debug variable 8 1 _a_data a_data
#line 232 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
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
#line 232 "D:\WORK\项目\派特\640\HT46R064B\HT46R064B.c"
@CODE .SECTION AT 0 'CODE'
CALL STARTUP
JMP begin
@DUMMY PROC
CALL _main
@DUMMY ENDP
