#include <vfd_buffered.h>

VFD_Buffered vfd(0, 1, 3);

/* Simple Buffer */
#define BUFFER_SIZE 256
static volatile uint8_t bufferHead = 0;
static volatile uint8_t bufferTail = 0;
static volatile uint8_t buffer[BUFFER_SIZE];

static const uint8_t thermometerFont[8] = {
  B00100,
  B01010,
  B01010,
  B01010,
  B10001,
  B10001,
  B01110,
  B00000,
};

static uint8_t bufferAvailable() {
  uint8_t head = bufferHead;
  uint8_t tail = bufferTail;

  if (head >= tail) {
    return (head - tail);
  }
  return BUFFER_SIZE + head - tail;
}

static uint8_t bufferRead() {
  uint8_t tail = bufferTail;
  uint8_t value = buffer[tail];
  if (++tail >= BUFFER_SIZE) {
    tail = 0;
  }
  bufferTail = tail;
  return value;
}

static uint16_t bufferRead16() {
  return bufferRead() + (bufferRead()<<8);
}

static uint32_t bufferRead32() {
  return bufferRead() + (bufferRead()<<8) + (bufferRead()<<16) + (bufferRead()<<24);
}

static void bufferWrite(uint8_t value) {
  uint8_t head = bufferHead;
  buffer[head] = value;
  if (++head >= BUFFER_SIZE) {
    head = 0;
  }
  bufferHead = head;
}

/* END Simple Buffer */

void yield() {
}

void setup() {
  Serial.begin(9600); // USB is always 12 Mbit/sec

  vfd.setCustomCharacterFont(0, thermometerFont);
  vfd.writeString (" Disco Easy Metrics ");
  vfd.writeString ("        v0.2        ");
  vfd.show();

  // Init SPI
  // Enable clock to GPIO Port C for SPI0 to operate
  SIM_SCGC5 |= SIM_SCGC5_PORTC;
  
  // Enable clock to to SPI0 module
  SIM_SCGC6 |= SIM_SCGC6_SPI0;

  // STOP SPI
  SPI0_MCR |= SPI_MCR_HALT | SPI_MCR_MDIS;
  
  // Configure SSI0 as SLAVE, continuous SCK enable, PCS0 active LOW, clear FIFOs
  SPI0_MCR = (SPI_MCR_CONT_SCKE | SPI_MCR_PCSIS(0x1F) | SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF | SPI_MCR_MTFE);

  // FrameSize=8bit, CPOL=0, CPHA=1
  SPI0_CTAR0_SLAVE = SPI_CTAR_FMSZ(7) | SPI_CTAR_CPOL;

  // Reieve FIFO drain request enable + Interrupt
  SPI0_RSER = SPI_RSER_RFDF_RE;

  // Set MUX and high drive strength enable
  PORTC_PCR4 = (PORT_PCR_MUX(2)| PORT_PCR_DSE);
  PORTC_PCR5 = (PORT_PCR_MUX(2)| PORT_PCR_DSE);
  PORTC_PCR6 = (PORT_PCR_MUX(2)| PORT_PCR_DSE);
  PORTC_PCR7 = (PORT_PCR_MUX(2)| PORT_PCR_DSE);

  // START SPI
  SPI0_MCR &= (~SPI_MCR_HALT & ~SPI_MCR_MDIS);

  // Start interrupt
  NVIC_ENABLE_IRQ(IRQ_SPI0);
}

void spi0_isr(void){
  //clear interrupt by writing a 1 so RFDF bit
  SPI0_SR |= SPI_SR_RFDF;
  while(SPI0_SR & 0xF0) {
    bufferWrite(SPI0_POPR);
  }
}

/* Global State */

uint8_t currTemp = 0;
uint8_t wantedTemp = 0;

uint16_t currLayer = 0;
uint16_t totalLayer = 0;

uint32_t printTime = 0;
uint32_t totalTime = 1;

uint32_t layerPrintTime = 0;
uint32_t layerTotalTime = 1;

bool durty = 0;
      
bool decodeData(uint8_t opCode) {
switch (opCode) {
    case 0x01:
      if (bufferAvailable() < 2) return false;
      // Ignore it for the moment
      bufferRead();
      bufferRead();
      break;
    case 0x10:
      if (bufferAvailable() < 2) return false;
      currTemp = bufferRead();
      wantedTemp = bufferRead();
      break;
    case 0x20:
      if (bufferAvailable() < 20) return false;
      printTime = bufferRead32();
      totalTime = bufferRead32();
      currLayer = bufferRead16();
      totalLayer = bufferRead16();
      layerPrintTime = bufferRead32();
      layerTotalTime = bufferRead32();
      break;
    case 0x28:
      if (bufferAvailable() < 1) return false;
      // Drop it for now
      bufferRead();
      break;
    case 0x29:
      if (bufferAvailable() < 1) return false;
      // Drop it for now
      bufferRead();
      break;
    default:
      durty = 1;
      // SKIP IT ?? Should not happens...
      break;
  }
  return true;
}

void loop(void) {
  static int opCode = -1;
  static bool needUpdate = false;

  // Handle serial buffer
  if (bufferAvailable()) {
    // Something to read, read opCode if not already done
    if (opCode == -1) {
      opCode = bufferRead();
    }

    if (decodeData(opCode)) {
      needUpdate = true;
      opCode = -1;
    }
  }

  if (needUpdate) {
      char buf[64];
      needUpdate = false;

      vfd.clearScreen();
      sprintf(buf, "\x8%d\x1A L:%d/%d", currTemp, currLayer+1, totalLayer);
      vfd.writeString(buf);
      sprintf(buf, "L:%.01f%% P:%.01f%% - %d", 100*layerPrintTime/(float)layerTotalTime, 100*printTime/(float)totalTime, durty);
      vfd.setPos(1,0);
      vfd.writeString(buf);
      vfd.show();
  }
}

