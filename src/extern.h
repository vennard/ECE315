//****
// library
//****
extern void PLL_Init(void);
extern bool initializeGPIOPort(uint32_t base, GPIO_CONFIG * init);
extern void DisableInterrupts(void);
extern void EnableInterrupts(void);

//****
// extern.c
//****
extern volatile struct MotorData {
	// PWM param
	int16_t pwm;
	// control params
	int32_t targ;
	int32_t speed;
	int32_t errsum;
} motor0, motor1;
extern volatile struct EncoderData {
	int32_t odo;
	uint8_t pos;
} enc0, enc1;
extern volatile struct UrfData {
	uint32_t s;
	uint32_t e;
	uint32_t dist;
} urf0, urf1;
extern volatile struct IrData {
	uint32_t dist;
} ir0;
extern volatile struct JoystickData {
	int32_t x;
	int32_t y;
	char press;
	char release;
} jstick0; 
extern volatile uint16_t adc0read[3];
// tables
extern char invgrey[4];
extern struct JsRange {
	int bot;
	int top;
} jsranges[];
extern volatile struct NavData {
	int left;
	int right;
} nav0; 


//****
// uart.c
//****
bool initDbgUART(uint32_t baudrate);
int txDbgUART(char *d);
int txDataDbgUART(char *d, uint32_t n);
int rxCharDbgUART(void);

bool initXbUART(uint32_t baudrate);
int txXbUART(char *d);
int txDataXbUART(char *d, uint32_t n);
int rxCharXbUART(void);


//****
// gpio.c
//****
extern GPIO_CONFIG portA_config;
extern GPIO_CONFIG portB_config;
extern GPIO_CONFIG portC_config;
extern GPIO_CONFIG portD_config;
extern GPIO_CONFIG portE_config;
extern GPIO_CONFIG portF_config;
