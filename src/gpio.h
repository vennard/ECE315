#ifndef __GPIO_H__
#define __GPIO_H__

#include <stdint.h>
#include <stdbool.h>

typedef struct {
  volatile uint32_t    Unused0[255];
  volatile uint32_t    Data;                    // + 0x3FC
  volatile uint32_t    Direction;               // + 0x400
  volatile uint32_t    InterruptSence;          // + 0x404
  volatile uint32_t    InterruptBothEdges;      // + 0x408
  volatile uint32_t    InterruptEvent;          // + 0x40C
  volatile uint32_t    InterruptMask;           // + 0x410
  volatile uint32_t    RawInterruptStatus;      // + 0x414
  volatile uint32_t    MaskedInterruptStatus;   // + 0x418
  volatile uint32_t    InterruptClear;          // + 0x41C
  volatile uint32_t    AlternateFunctionSelect; // + 0x420
  volatile uint32_t    Unused1[55];     
  volatile uint32_t    DriveSelect2mA;          // + 0x500
  volatile uint32_t    DriveSelect4mA;          // + 0x504
  volatile uint32_t    DriveSelect8mA;          // + 0x508
  volatile uint32_t    OpenDrainSelect;         // + 0x50C
  volatile uint32_t    PullUpSelect;            // + 0x510
  volatile uint32_t    PullDownSelect;          // + 0x514
  volatile uint32_t    SlewRateControlSelect;   // + 0x518
  volatile uint32_t    DigitalEnable;           // + 0x51C
  volatile uint32_t    Lock;                    // + 0x520
  volatile uint32_t    Commit;                  // + 0x524
  volatile uint32_t    AnalogSelectMode;        // + 0x528
  volatile uint32_t    PortControl;             // + 0x52C
  volatile uint32_t    ADCControl;              // + 0x530
} GPIO_PORT;

typedef struct {
  uint8_t     DigitalEnable;
  uint8_t     Input;
  uint8_t     Output;
  uint8_t     InterruptEnable;
  uint8_t     InterruptLevel;
  uint8_t     InterruptLevelActiveHigh;
  uint8_t     InterruptEdge;
  uint8_t     InterruptEdgeRising;
  uint8_t     InterruptEdgeBoth;
  uint8_t     PullDown;
  uint8_t     PullUp;
  uint8_t     AnalogEnable;
  uint8_t     AlternateFunctionEnable;
  uint32_t    PortControl;
} GPIO_CONFIG;

#endif
