#include <Wire.h>
#include "adi_a2b_i2c_commandlist.h"
#include "adi_adau1452_i2c_commandlist.h"
#include "adi_adau1761_i2c_commandlist.h"

// Master node I2C address
#define A2B_MASTER_ADDR 0x68

// Chip registers
#define VENDOR 0x02
#define PRODUCT 0x03
#define VERSION 0x04
#define CAPABILITY 0x05
#define SWSTAT 0x14
#define INTSRC 0x16
#define INTTYPE 0x17

// Forms I2C packets for writing to the A2B bus
static void adi_a2b_Concat_Addr_Data(unsigned char pDstBuf[], unsigned int nAddrwidth, unsigned int nAddr)
{
	/* Store the read values in the place holder */
	switch (nAddrwidth)
	{ /* Byte */
		case 1u:
			pDstBuf[0u] = (unsigned char)nAddr;
			break;
			/* 16 bit word*/
		case 2u:

			pDstBuf[0u] = (unsigned char)(nAddr >> 8u);
			pDstBuf[1u] = (unsigned char)(nAddr & 0xFFu);

			break;
			/* 24 bit word */
		case 3u:
			pDstBuf[0u] = (unsigned char)((nAddr & 0xFF0000u) >> 16u);
			pDstBuf[1u] = (unsigned char)((nAddr & 0xFF00u) >> 8u);
			pDstBuf[2u] = (unsigned char)(nAddr & 0xFFu);
			break;

			/* 32 bit word */
		case 4u:
			pDstBuf[0u] = (unsigned char)(nAddr >> 24u);
			pDstBuf[1u] = (unsigned char)((nAddr & 0xFF0000u) >> 16u);
			pDstBuf[2u] = (unsigned char)((nAddr & 0xFF00u) >> 8u);
			pDstBuf[3u] = (unsigned char)(nAddr & 0xFFu);
			break;

		default:
			break;

	}
}

// Writes data to the I2C bus
static int i2cWrite(unsigned short devAddr, unsigned short count, unsigned char *bytes)
{
  digitalWrite(LED_STAT, LOW);
	Wire.beginTransmission(devAddr);
	Wire.write(bytes, count);
	return Wire.endTransmission();
  digitalWrite(LED_STAT, HIGH);
}

// Reads a single byte from the I2C bus
static byte i2cReadByte(unsigned short devAddr, unsigned short reg)
{
  digitalWrite(LED_STAT, LOW);
  Wire.beginTransmission(devAddr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(devAddr, 1);
  byte readData = Wire.read();
  digitalWrite(LED_STAT, HIGH);

  return readData;
}

static char* i2cReadBlock(unsigned short devAddr, unsigned short base, unsigned int len)
{
  digitalWrite(LED_STAT, LOW);
  Wire.beginTransmission(devAddr);
  Wire.write(base);
  Wire.endTransmission();
  Wire.requestFrom(devAddr, len);

  if (len > Wire.available()) {
    char readData[len];
  
    for (int i=0; i<len; i++) {
      readData[i] = Wire.read();
    }
  
    digitalWrite(LED_STAT, HIGH);

    return readData;
  }
}

// Parses operations defined in the command list header, and calls the appropriate functions
static void adi_a2b_NetworkSetup()
{
  
	ADI_A2B_DISCOVERY_CONFIG* pOPUnit;
	unsigned int nIndex, nIndex1;
	unsigned int status;
	/* Maximum number of writes */
	static unsigned char aDataBuffer[6000];
	static unsigned char aDataWriteReadBuf[4u];
	unsigned int nDelayVal;

	/* Loop over all the configuration */
	for (nIndex = 0; nIndex < CONFIG_LEN; nIndex++)
	{
		pOPUnit = &gaA2BConfig[nIndex];
		/* Operation code*/
		switch (pOPUnit->eOpCode)
		{
			/* Write */
			case WRITE:
				adi_a2b_Concat_Addr_Data(&aDataBuffer[0u], pOPUnit->nAddrWidth, pOPUnit->nAddr);
				(void)memcpy(&aDataBuffer[pOPUnit->nAddrWidth], pOPUnit->paConfigData, pOPUnit->nDataCount);
				/* printk("Operation number \n %d", nIndex);*/
				/* PAL Call, replace with custom implementation  */
				i2cWrite(pOPUnit->nDeviceAddr, (pOPUnit->nAddrWidth + pOPUnit->nDataCount), &aDataBuffer[0u]);
				break;

				/* Read */
			case READ:
				(void)memset(&aDataBuffer[0u], 0u, pOPUnit->nDataCount);
				adi_a2b_Concat_Addr_Data(&aDataWriteReadBuf[0u], pOPUnit->nAddrWidth, pOPUnit->nAddr);
				/* Couple of milli seconds should be OK */
				delay(2);
				break;

				/* Delay */
			case DELAY:
				nDelayVal = 0u;
				for (nIndex1 = 0u; nIndex1 < pOPUnit->nDataCount; nIndex1++)
				{
					nDelayVal = pOPUnit->paConfigData[nIndex1] | nDelayVal << 8u;
				}
				delay(nDelayVal);
				break;

			default:
				break;

		}
	}
}

int processInterrupt(uint8_t interrupt_src, uint8_t interrupt_type)
{
  uint8_t intSrcHigh = interrupt_src >> 4;
  uint8_t intSrcLow = interrupt_src & 0x0f;

  switch (intSrcHigh)
  {
    case 0x08:
      Serial.print("INFO: Slave ");
      Serial.print(intSrcLow);
      Serial.println(" interrupt");
      break;
    case 0x06:
      Serial.println("INFO: Master interrupt");
      break;
    default:
      break;
  }
  
  switch (interrupt_type)
  {
    case 0x0c:
      Serial.println("WARN: Cable disconnected, restarting discovery");
      adi_a2b_NetworkSetup();
      break;
    case 0x0f:
      Serial.println("WARN: Unknown power fault, restarting discovery");
      adi_a2b_NetworkSetup();
      break;
    case 0x72:
      Serial.println("INFO: A2B slave detected, restarting discovery");
      adi_a2b_NetworkSetup();
      break;
    case 0x24:
      Serial.println("INFO: Discovery finished");
      break;
    case 128:
      Serial.println("ERROR: A2B messaging error");
      break;
    case 0xff:
      Serial.println("INFO: Master node PLL locked");
      break;
    default:
      Serial.print("INFO: A2B interrupt received: ");
      Serial.println(interrupt_type);
      break;
  }
  return intSrcLow;
}

void printA2bMasterChipId() {

  if (i2cReadByte(A2B_MASTER_ADDR, VENDOR) == 0xad) {
    Serial.print("INFO: Detected");
    Serial.print(" AD24");
    Serial.print(i2cReadByte(A2B_MASTER_ADDR, PRODUCT), HEX);
    Serial.print(" version: ");
    Serial.print(i2cReadByte(A2B_MASTER_ADDR, VERSION), HEX);
    Serial.print(" capability: ");
    Serial.print(i2cReadByte(A2B_MASTER_ADDR, CAPABILITY), HEX);
    Serial.print(" at address: 0x");
    Serial.println(A2B_MASTER_ADDR, HEX);
  } else {
    Serial.println("ERROR: A2B master chip not detected");
  }
}

void setup() {
  pinMode(2, OUTPUT);
  Wire.begin();
  Serial.begin(115200);

  delay(500);
  Serial.println("INFO: ADAU1452 init");
  adau1452_init();
  delay(500);
  Serial.println("INFO: A2B init");
  printA2bMasterChipId();
  adi_a2b_NetworkSetup();
  delay(500);
  Serial.println("INFO: ADAU1761 init");
  adau1761_init();
}

void loop() {
  byte intSrc = i2cReadByte(A2B_MASTER_ADDR, INTSRC);
  byte intType = i2cReadByte(A2B_MASTER_ADDR, INTTYPE);
  byte swStatus = i2cReadByte(A2B_MASTER_ADDR, SWSTAT);

  Serial.print("DEBUG: SWSTAT: ");
  Serial.print(swStatus, HEX);
  Serial.print(" INTSRC: ");
  Serial.print(intSrc, HEX);
  Serial.print(" INTTYPE: ");
  Serial.println(intType, HEX);
  
  
  if (intType == 0x00)
  {
    if (swStatus != 0xff)
    {
      Serial.println("WARN: A2B network down! Restarting discovery");
      digitalWrite(LED_BUILTIN, LOW);
      delay(1000);
      adi_a2b_NetworkSetup();
    }
    delay(1000);
  } else
  {
    processInterrupt(intSrc, intType);
  }
  delay(1000);
}
