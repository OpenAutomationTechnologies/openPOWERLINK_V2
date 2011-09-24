# define COMPUTED_PI_OUT_SIZE 4
typedef struct 
{
	unsigned CN1_M00_Digital_Input_8_Bit_Byte_1:8;
	unsigned CN32_M00_Digital_Input_8_Bit_Byte_1:8;
	unsigned CN110_M00_Digital_Input_8_Bit_Byte_1:8;
	unsigned PADDING_VAR_1:8;
} PI_OUT;

# define COMPUTED_PI_IN_SIZE 4
typedef struct 
{
	unsigned CN1_M00_Digital_Ouput_8_Bit_Byte_1:8;
	unsigned CN32_M00_Digital_Ouput_8_Bit_Byte_1:8;
	unsigned CN110_M00_Digital_Ouput_8_Bit_Byte_1:8;
	unsigned PADDING_VAR_1:8;
} PI_IN;
