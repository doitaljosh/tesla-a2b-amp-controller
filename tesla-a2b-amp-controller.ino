#include <Wire.h>
#include "adi_a2b_i2c_commandlist.h"

// Master node I2C address
#define A2B_MASTER_ADDR 0x68

// Chip registers
#define INTSRC 0x16
#define INTTYPE 0x17
#define SWSTAT 0x14

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
static int adi_a2b_I2CWrite(unsigned short devAddr, unsigned short count, unsigned char *bytes)
{
	Wire.beginTransmission(devAddr);
	Wire.write(bytes, count);
	Wire.endTransmission();
}

// Reads a single byte from the I2C bus
static byte adi_a2b_I2CReadByte(unsigned short devAddr, unsigned short reg)
{
  Wire.beginTransmission(devAddr);
  Wire.write(reg);
  Wire.requestFrom(reg, 1);
  byte readData = Wire.read();
  Wire.endTransmission();

  return readData;
}

// Parses operations defined in the command list header, and calls the appropriate functions
static void adi_a2b_NetworkSetup()
{
  digitalWrite(LED_BUILTIN, HIGH);
  
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
				adi_a2b_I2CWrite(pOPUnit->nDeviceAddr, (pOPUnit->nAddrWidth + pOPUnit->nDataCount), &aDataBuffer[0u]);
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

void processInterrupt(uint16_t interrupt)
{
  switch (interrupt)
  {
    case 0x01:
      Serial.println("A2B network is up");
      break;
    case 0x72:
      Serial.println("A2B slave detected, restarting discovery");
      adi_a2b_NetworkSetup();
      break;
    case 0x9e:
      Serial.println("Discovery finished");
      digitalWrite(LED_BUILTIN, LOW);
      break;
    case 128:
      Serial.println("A2B messaging error");
      break;
    case 0xff:
      Serial.println("A2B master not detected");
      break;
    default:
      Serial.print("A2B interrupt received: ");
      Serial.println(interrupt);
      break;
  }

  if (interrupt & 0x08)
  {
    Serial.println("Master interrupt");
  } else if (interrupt & 0x04)
  {
    Serial.println("Slave interrupt");
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin();
  Serial.begin(115200);

  adi_a2b_NetworkSetup();
}

void loop() {
  byte intSrc = adi_a2b_I2CReadByte(A2B_MASTER_ADDR, INTSRC);
  byte intType = adi_a2b_I2CReadByte(A2B_MASTER_ADDR, INTTYPE);
  byte swStatus = adi_a2b_I2CReadByte(A2B_MASTER_ADDR, SWSTAT);

  Serial.print("SWSTAT: ");
  Serial.println(swStatus);
  Serial.print("INTTYPE: ");
  Serial.println(intType);
  
  if (intType == 0x00)
  {
    if (swStatus != 0xff)
    {
      Serial.println("A2B network down! Restarting discovery");
      digitalWrite(LED_BUILTIN, LOW);
      delay(1000);
      adi_a2b_NetworkSetup();
    }
    delay(1000);
  } else
  {
    processInterrupt(intType);
  }
  delay(1000);
}
