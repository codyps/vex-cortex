/* Documetaion of the VEX interfaces */

struct oi_data {
	// right stick
	u8 axis_1;
	u8 axis_2;
	
	// left stick
	u8 axis_3;
	u8 axis_4;
	
	// accelerometer
	u8 accel_x;
	u8 accel_y;
	u8 accel_z;
	
	// trigger buttons ( comment data given is clearly incorrect)
	/* 
	byte 7 = Group 5 & 6   //Buttons  
		bit0 = Left Down       *** Group 5
		bit1 = Left Up
		bit2 = Right Down      *** Group 6
		bit3 = Left Down
	*/
	u8 group_5a:1;
	u8 group_5b:1;
	u8 group_6a:1;
	u8 group_6b:1;
	u8 reserved1:4; // not mentioned.
	
	u8 g8_down:1;
	u8 g8_left:1;
	u8 g8_up:1;
	u8 g8_right:1;
	
	u8 g7_down:1;
	u8 g7_left:1;
	u8 g7_up:1;
	u8 g7_right:1;
	
	u8 reserved2[3]; // noted as "spare"
};

struct state_pack {
	u8 iack:1;
	u8 config:1;
	u8 initializing:1; // data is not ready.
	u8 valid:1; // data is valid
	u8 reserved:4;
};

/* SPI (uses SPI1)
*Initialization Process:
PE0 set high ("SPI1_INT")
read 8 bytes of junk data.
wait for PE3 && PE4 to be low.



*Tranfers
Transfers are triggered every 20ms.
32 bytes are transmitted for every transfer.
PA11 set high, called "RTS". ("RTS high Used to ensure 1st 4 bytes").
For each byte:
	PE1 set low; appears to be slave select. ("Use as SSL0")
	Reads one byte.
	Writes one byte.
	PE1 set high.
	Claims to wait 15us (uses "for(i=0;i<150;i++)")
	if the byte number is a multiple of 4 (this is what the code does.
	  comments, however, indicate that this should only occour after the 4th byte):
		They "Make a gap" (sersiously?)
		delay 210us ("for(i=0;i<1000;i++)")
		PA11 ("RTS") set low.
packet num (in the slave packet) is incremented following each transfer.
*/

#define SYNC_MAGIC 0xC917

//Data From Master
typedef struct { 
	u16 sync; // Should always be SYNC_MAGIC
	union {
		u8  a;
		struct state_pack b;
	} state;
	union {
		u8 a;
		struct {
			u8 tx1_active:1;
			u8 tx2_active:1;
			u8 spare:1;
			u8 competition_mode:1; //XXX: what does this imply?
			u8 reset_slave:1; //XXX: noted as "(Reserved)" but has a name.
			u8 joystick_mode:1; //XXX: wtf is joystick mode?
			u8 autonomus:1;
			u8 disable:1;
		} b;
	} SystemFlags;
	u8  mainBatteryVoltage; // mult by 0.0591 for something readable.      
	u8  backupBatteryVoltage;
	union {
		u8  a[12];
		struct oi_data b;
	} joystick[2];
	u8  version;
	u8  packetNum;
} master_spi_packet;

//Data To Master
typedef struct { 
	u16 sync; // should always be SYNC_MAGIC
	union {
		u8 a;
		struct state_pack b;
	} state;
	union {
		u8 a;
		struct {
			u8 auton:1;
			u8 crystal_mode:1;
			u8 disable:1;
			u8 brake:1; //XXX: what does this mean?
			u8 enable_printfs:1; //XXX: noted as "Reserved for Master"
			u8 enable_display:1; //XXX: noted as "Reserved for Master"
			u8 reserved:2; // unmentioned.
		} b;
	} SystemFlags; //XXX: "Reserved for Slave (TBD)"
	u8  DigitalByte1;   //Digital bits 1-8      
	u8  DigitalByte2;   //Digital bits 9-12, 13-16 (spare)   
	u8  Motor[8];       //PWM values 0-255
	u8  MotorStatus[8]; //XXX: "PWM motor states (TBD)"
	u8  Analog[8];      //Analog port (1-8)
	u8  version;    
	u8  packetNum;    
} slave_spi_packet;

/* Crystal Detection
PB10: low when RX1 is connected.
PC8 : low when RX2 is connected.
*/

/* Control Pins for Motors 1 & 10. Connected directly to the STM.
// See Set_MotorControl_Sw{1,2} for details.
// (These use timer4 in the default code)
// Active low. Names taken from original comments.
PD3 : AH1 // gpio
PD4 : BH1 // gpio
PD7 : AH2 // gpio
PD8 : BH2 // gpio
PD12: AL1
PD13: BL1
PD14: AL2 
PD15: BL2
*/

/* Digital IO
PE9 : 1
PE11: 2
PC6 : 3
PC7 : 4
PE13: 5
PE14: 6
PE8 : 7
PE10: 8
PE12: 9
PE7 : 10
PD0 : 11
PD1 : 12
*/
