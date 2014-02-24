
bool initDbgUART(uint32_t baudrate);
int txDbgUART(char *d, bool block);
int txDataDbgUART(char *d, uint32_t n, bool block);
int rxCharDbgUART(bool block);

bool initXbUART(uint32_t baudrate);
int txXbUART(char *d, bool block);
int txDataXbUART(char *d, uint32_t n, bool block);
int rxCharXbUART(bool block);

extern GPIO_CONFIG portA_config;
extern GPIO_CONFIG portB_config;
extern GPIO_CONFIG portC_config;
extern GPIO_CONFIG portD_config;
extern GPIO_CONFIG portE_config;
extern GPIO_CONFIG portF_config;
