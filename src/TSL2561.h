/*
 MODIFIED BY: @indefinit, 2018
 intended for RPi3 instead of Arduino
 */
/*!
 * @file Adafruit_TSL2561_U.h
 *
 * This is part of Adafruit's FXOS8700 driver for the Arduino platform.  It is
 * designed specifically to work with the Adafruit FXOS8700 breakout:
 * https://www.adafruit.com/products/3463
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Kevin "KTOWN" Townsend for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#pragma once

#include "Adafruit_Sensor.h"
#include "ofxGPIO.h"

#define TSL2561_VISIBLE 2                   ///< channel 0 - channel 1
#define TSL2561_INFRARED 1                  ///< channel 1
#define TSL2561_FULLSPECTRUM 0              ///< channel 0

// I2C address options
#define TSL2561_ADDR_LOW          (0x29)    ///< Default address (pin pulled low)
#define TSL2561_ADDR_FLOAT        (0x39)    ///< Default address (pin left floating)
#define TSL2561_ADDR_HIGH         (0x49)    ///< Default address (pin pulled high)

// Lux calculations differ slightly for CS package
//#define TSL2561_PACKAGE_CS                ///< Chip scale package
#define TSL2561_PACKAGE_T_FN_CL             ///< Dual Flat No-Lead package

#define TSL2561_COMMAND_BIT       (0x80)    ///< Must be 1
#define TSL2561_CLEAR_BIT         (0x40)    ///< Clears any pending interrupt (write 1 to clear)
#define TSL2561_WORD_BIT          (0x20)    ///< 1 = read/write word (rather than byte)
#define TSL2561_BLOCK_BIT         (0x10)    ///< 1 = using block read/write

#define TSL2561_CONTROL_POWERON   (0x03)    ///< Control register setting to turn on
#define TSL2561_CONTROL_POWEROFF  (0x00)    ///< Control register setting to turn off

#define TSL2561_LUX_LUXSCALE      (14)      ///< Scale by 2^14
#define TSL2561_LUX_RATIOSCALE    (9)       ///< Scale ratio by 2^9
#define TSL2561_LUX_CHSCALE       (10)      ///< Scale channel values by 2^10
#define TSL2561_LUX_CHSCALE_TINT0 (0x7517)  ///< 322/11 * 2^TSL2561_LUX_CHSCALE
#define TSL2561_LUX_CHSCALE_TINT1 (0x0FE7)  ///< 322/81 * 2^TSL2561_LUX_CHSCALE

// T, FN and CL package values
#define TSL2561_LUX_K1T           (0x0040)  ///< 0.125 * 2^RATIO_SCALE
#define TSL2561_LUX_B1T           (0x01f2)  ///< 0.0304 * 2^LUX_SCALE
#define TSL2561_LUX_M1T           (0x01be)  ///< 0.0272 * 2^LUX_SCALE
#define TSL2561_LUX_K2T           (0x0080)  ///< 0.250 * 2^RATIO_SCALE
#define TSL2561_LUX_B2T           (0x0214)  ///< 0.0325 * 2^LUX_SCALE
#define TSL2561_LUX_M2T           (0x02d1)  ///< 0.0440 * 2^LUX_SCALE
#define TSL2561_LUX_K3T           (0x00c0)  ///< 0.375 * 2^RATIO_SCALE
#define TSL2561_LUX_B3T           (0x023f)  ///< 0.0351 * 2^LUX_SCALE
#define TSL2561_LUX_M3T           (0x037b)  ///< 0.0544 * 2^LUX_SCALE
#define TSL2561_LUX_K4T           (0x0100)  ///< 0.50 * 2^RATIO_SCALE
#define TSL2561_LUX_B4T           (0x0270)  ///< 0.0381 * 2^LUX_SCALE
#define TSL2561_LUX_M4T           (0x03fe)  ///< 0.0624 * 2^LUX_SCALE
#define TSL2561_LUX_K5T           (0x0138)  ///< 0.61 * 2^RATIO_SCALE
#define TSL2561_LUX_B5T           (0x016f)  ///< 0.0224 * 2^LUX_SCALE
#define TSL2561_LUX_M5T           (0x01fc)  ///< 0.0310 * 2^LUX_SCALE
#define TSL2561_LUX_K6T           (0x019a)  ///< 0.80 * 2^RATIO_SCALE
#define TSL2561_LUX_B6T           (0x00d2)  ///< 0.0128 * 2^LUX_SCALE
#define TSL2561_LUX_M6T           (0x00fb)  ///< 0.0153 * 2^LUX_SCALE
#define TSL2561_LUX_K7T           (0x029a)  ///< 1.3 * 2^RATIO_SCALE
#define TSL2561_LUX_B7T           (0x0018)  ///< 0.00146 * 2^LUX_SCALE
#define TSL2561_LUX_M7T           (0x0012)  ///< 0.00112 * 2^LUX_SCALE
#define TSL2561_LUX_K8T           (0x029a)  ///< 1.3 * 2^RATIO_SCALE
#define TSL2561_LUX_B8T           (0x0000)  ///< 0.000 * 2^LUX_SCALE
#define TSL2561_LUX_M8T           (0x0000)  ///< 0.000 * 2^LUX_SCALE

// CS package values
#define TSL2561_LUX_K1C           (0x0043)  ///< 0.130 * 2^RATIO_SCALE
#define TSL2561_LUX_B1C           (0x0204)  ///< 0.0315 * 2^LUX_SCALE
#define TSL2561_LUX_M1C           (0x01ad)  ///< 0.0262 * 2^LUX_SCALE
#define TSL2561_LUX_K2C           (0x0085)  ///< 0.260 * 2^RATIO_SCALE
#define TSL2561_LUX_B2C           (0x0228)  ///< 0.0337 * 2^LUX_SCALE
#define TSL2561_LUX_M2C           (0x02c1)  ///< 0.0430 * 2^LUX_SCALE
#define TSL2561_LUX_K3C           (0x00c8)  ///< 0.390 * 2^RATIO_SCALE
#define TSL2561_LUX_B3C           (0x0253)  ///< 0.0363 * 2^LUX_SCALE
#define TSL2561_LUX_M3C           (0x0363)  ///< 0.0529 * 2^LUX_SCALE
#define TSL2561_LUX_K4C           (0x010a)  ///< 0.520 * 2^RATIO_SCALE
#define TSL2561_LUX_B4C           (0x0282)  ///< 0.0392 * 2^LUX_SCALE
#define TSL2561_LUX_M4C           (0x03df)  ///< 0.0605 * 2^LUX_SCALE
#define TSL2561_LUX_K5C           (0x014d)  ///< 0.65 * 2^RATIO_SCALE
#define TSL2561_LUX_B5C           (0x0177)  ///< 0.0229 * 2^LUX_SCALE
#define TSL2561_LUX_M5C           (0x01dd)  ///< 0.0291 * 2^LUX_SCALE
#define TSL2561_LUX_K6C           (0x019a)  ///< 0.80 * 2^RATIO_SCALE
#define TSL2561_LUX_B6C           (0x0101)  ///< 0.0157 * 2^LUX_SCALE
#define TSL2561_LUX_M6C           (0x0127)  ///< 0.0180 * 2^LUX_SCALE
#define TSL2561_LUX_K7C           (0x029a)  ///< 1.3 * 2^RATIO_SCALE
#define TSL2561_LUX_B7C           (0x0037)  ///< 0.00338 * 2^LUX_SCALE
#define TSL2561_LUX_M7C           (0x002b)  ///< 0.00260 * 2^LUX_SCALE
#define TSL2561_LUX_K8C           (0x029a)  ///< 1.3 * 2^RATIO_SCALE
#define TSL2561_LUX_B8C           (0x0000)  ///< 0.000 * 2^LUX_SCALE
#define TSL2561_LUX_M8C           (0x0000)  ///< 0.000 * 2^LUX_SCALE

// Auto-gain thresholds
#define TSL2561_AGC_THI_13MS      (4850)    ///< Max value at Ti 13ms = 5047
#define TSL2561_AGC_TLO_13MS      (100)     ///< Min value at Ti 13ms = 100
#define TSL2561_AGC_THI_101MS     (36000)   ///< Max value at Ti 101ms = 37177
#define TSL2561_AGC_TLO_101MS     (200)     ///< Min value at Ti 101ms = 200
#define TSL2561_AGC_THI_402MS     (63000)   ///< Max value at Ti 402ms = 65535
#define TSL2561_AGC_TLO_402MS     (500)     ///< Min value at Ti 402ms = 500

// Clipping thresholds
#define TSL2561_CLIPPING_13MS     (4900)    ///< # Counts that trigger a change in gain/integration
#define TSL2561_CLIPPING_101MS    (37000)   ///< # Counts that trigger a change in gain/integration
#define TSL2561_CLIPPING_402MS    (65000)   ///< # Counts that trigger a change in gain/integration

// Delay for integration times
#define TSL2561_DELAY_INTTIME_13MS    (15)    ///< Wait 15ms for 13ms integration
#define TSL2561_DELAY_INTTIME_101MS   (120)   ///< Wait 120ms for 101ms integration
#define TSL2561_DELAY_INTTIME_402MS   (450)   ///< Wait 450ms for 402ms integration

/** TSL2561 I2C Registers */
enum
{
	TSL2561_REGISTER_CONTROL          = 0x00, // Control/power register
	TSL2561_REGISTER_TIMING           = 0x01, // Set integration time register
	TSL2561_REGISTER_THRESHHOLDL_LOW  = 0x02, // Interrupt low threshold low-byte
	TSL2561_REGISTER_THRESHHOLDL_HIGH = 0x03, // Interrupt low threshold high-byte
	TSL2561_REGISTER_THRESHHOLDH_LOW  = 0x04, // Interrupt high threshold low-byte
	TSL2561_REGISTER_THRESHHOLDH_HIGH = 0x05, // Interrupt high threshold high-byte
	TSL2561_REGISTER_INTERRUPT        = 0x06, // Interrupt settings
	TSL2561_REGISTER_CRC              = 0x08, // Factory use only
	TSL2561_REGISTER_ID               = 0x0A, // TSL2561 identification setting
	TSL2561_REGISTER_CHAN0_LOW        = 0x0C, // Light data channel 0, low byte
	TSL2561_REGISTER_CHAN0_HIGH       = 0x0D, // Light data channel 0, high byte
	TSL2561_REGISTER_CHAN1_LOW        = 0x0E, // Light data channel 1, low byte
	TSL2561_REGISTER_CHAN1_HIGH       = 0x0F  // Light data channel 1, high byte
};

/** Three options for how long to integrate readings for */
typedef enum
{
	TSL2561_INTEGRATIONTIME_13MS      = 0x00,    // 13.7ms
	TSL2561_INTEGRATIONTIME_101MS     = 0x01,    // 101ms
	TSL2561_INTEGRATIONTIME_402MS     = 0x02     // 402ms
}
tsl2561IntegrationTime_t;

/** TSL2561 offers 2 gain settings */
typedef enum
{
	TSL2561_GAIN_1X                   = 0x00,    // No gain
	TSL2561_GAIN_16X                  = 0x10,    // 16x gain
}
tsl2561Gain_t;



/**************************************************************************/
/*!
 @brief  Class that stores state and functions for interacting with TSL2561 Light Sensor
 */
/**************************************************************************/
class Adafruit_TSL2561_Unified : public Adafruit_Sensor {
public:
	/**************************************************************************/
	/*!
	 @brief Constructor
	 @param addr The I2C address this chip can be found on, 0x29, 0x39 or 0x49
	 @param sensorID An optional ID that will be placed in sensor events to help
	 keep track if you have many sensors in use
	 */
	/**************************************************************************/
	Adafruit_TSL2561_Unified(uint8_t addr=TSL2561_ADDR_FLOAT, int32_t sensorID = -1):	_addr(addr),_tsl2561Initialised(false),_tsl2561AutoGain(false),_tsl2561IntegrationTime(TSL2561_INTEGRATIONTIME_13MS),_tsl2561Gain(TSL2561_GAIN_1X), _tsl2561SensorID(sensorID)
	{}
	
	/**************************************************************************/
	/*!
	 @brief Initializes I2C and configures the sensor with default Wire I2C
	 (call this function before doing anything else)
	 @returns True if sensor is found and initialized, false otherwise.
	 */
	/**************************************************************************/
	void setup(void){
		_i2c = std::make_shared<I2c>("/dev/i2c-1");//default i2c bus
		_i2c->addressSet(_addr);//0x39
		init();
	}
	/**************************************************************************/
	/*!
	 @brief Initializes I2C and configures the sensor with provided I2C device
	 (call this function before doing anything else)
	 @param theWire A pointer to any I2C interface (e.g. &Wire1)
	 @returns True if sensor is found and initialized, false otherwise.
	 */
	/**************************************************************************/
	void setup(std::shared_ptr<I2c> i2c){
		_i2c = i2c;
		init();
	}
	/**************************************************************************/
	/*!
	 @brief  Initializes I2C connection and settings.
	 Attempts to determine if the sensor is contactable, then sets up a default
	 integration time and gain. Then powers down the chip.
	 @returns True if sensor is found and initialized, false otherwise.
	 */
	/**************************************************************************/
	void init(){
		// Power ON mode(0x03)
		char config[2] = {0};
		config[0] = 0x00 | 0x80;
		config[1] = 0x03;
		_i2c->writeByte((uint8_t) config[0],(uint8_t)config[1]);
		
		// Select timing register(0x01 | 0x80)
		// Nominal integration time = 402ms(0x02)
		config[0] = 0x01 | 0x80;
		config[1] = 0x02;
		_i2c->writeByte((uint8_t) config[0], config[1]);
		usleep(100);

		_tsl2561Initialised = true;
		
		/* Set default integration time and gain */
		//setIntegrationTime(_tsl2561IntegrationTime);
		//setGain(_tsl2561Gain);
	}
	
	/* TSL2561 Functions */
	/**************************************************************************/
	/*!
	 @brief  Enables or disables the auto-gain settings when reading
	 data from the sensor
	 @param enable Set to true to enable, False to disable
	 */
	/**************************************************************************/
	void enableAutoRange(bool enable){
		_tsl2561AutoGain = enable ? true : false;
	}
	/**************************************************************************/
	/*!
	 @brief      Sets the integration time for the TSL2561. Higher time means
	 more light captured (better for low light conditions) but will
	 take longer to run readings.
	 @param time The amount of time we'd like to add up values
	 */
	/**************************************************************************/
	void setIntegrationTime(tsl2561IntegrationTime_t time){
		if (!_tsl2561Initialised) init();
		
		/* Enable the device by setting the control bit to 0x03 */
		enable();
		
		/* Update the timing register */
		write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_TIMING, time | _tsl2561Gain);
		
		/* Update value placeholders */
		_tsl2561IntegrationTime = time;
		
		/* Turn the device off to save power */
		disable();
	}
	/**************************************************************************/
	/*!
	 @brief  Adjusts the gain on the TSL2561 (adjusts the sensitivity to light)
	 @param gain The value we'd like to set the gain to
	 */
	/**************************************************************************/
	void setGain(tsl2561Gain_t gain){
		if (!_tsl2561Initialised) init();
		
		/* Enable the device by setting the control bit to 0x03 */
		enable();
		
		/* Update the timing register */
		write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_TIMING, _tsl2561IntegrationTime | gain);
		
		/* Update value placeholders */
		_tsl2561Gain = gain;
		
		/* Turn the device off to save power */
		disable();
	}
	
	float getFullSpectrum() {
		// Read 4 bytes of data from register(0x0C | 0x80)
		// ch0 lsb, ch0 msb, ch1 lsb, ch1 msb
		char reg = 0x0C | 0x80;
		uint8_t data[4] = {0};
		pBus->readBlock(reg, 4, data);
		// Convert the data
		return (data[1] * 256 + data[0]);
	}
	
	float getIR() {
		// Read 4 bytes of data from register(0x0C | 0x80)
		// ch0 lsb, ch0 msb, ch1 lsb, ch1 msb
		char reg = 0x0C | 0x80;
		uint8_t data[4] = {0};
		pBus->readBlock(reg, 4, data);
		// Convert the data
		return (data[3] * 256 + data[2]);
	}
	
	float getVisible(){
		// Read 4 bytes of data from register(0x0C | 0x80)
		// ch0 lsb, ch0 msb, ch1 lsb, ch1 msb
		char reg = 0x0C | 0x80;
		uint8_t data[4] = {0};
		pBus->readBlock(reg, 4, data);
		// Convert the data
		float ch0 = (data[1] * 256 + data[0]);
		float ch1 = (data[3] * 256 + data[2]);
		return ch0-ch1;
	}
	/**************************************************************************/
	/*!
	 @brief  Gets the broadband (mixed lighting) and IR only values from
	 the TSL2561, adjusting gain if auto-gain is enabled
	 @param  broadband Pointer to a uint16_t we will fill with a sensor
	 reading from the IR+visible light diode.
	 @param  ir Pointer to a uint16_t we will fill with a sensor the
	 IR-only light diode.
	 */
	/**************************************************************************/

	void getLuminosity (uint16_t *broadband, uint16_t *ir){
		bool valid = false;
		
		if (!_tsl2561Initialised) init();
		
		/* If Auto gain disabled get a single reading and continue */
		if(!_tsl2561AutoGain)
		{
			getData (broadband, ir);
			return;
		}
		
		/* Read data until we find a valid range */
		bool _agcCheck = false;
		do
		{
			uint16_t _b, _ir;
			uint16_t _hi, _lo;
			tsl2561IntegrationTime_t _it = _tsl2561IntegrationTime;
			
			/* Get the hi/low threshold for the current integration time */
			switch(_it)
			{
				case TSL2561_INTEGRATIONTIME_13MS:
					_hi = TSL2561_AGC_THI_13MS;
					_lo = TSL2561_AGC_TLO_13MS;
					break;
				case TSL2561_INTEGRATIONTIME_101MS:
					_hi = TSL2561_AGC_THI_101MS;
					_lo = TSL2561_AGC_TLO_101MS;
					break;
				default:
					_hi = TSL2561_AGC_THI_402MS;
					_lo = TSL2561_AGC_TLO_402MS;
					break;
			}
			
			getData(&_b, &_ir);
			
			/* Run an auto-gain check if we haven't already done so ... */
			if (!_agcCheck)
			{
				if ((_b < _lo) && (_tsl2561Gain == TSL2561_GAIN_1X))
				{
					/* Increase the gain and try again */
					setGain(TSL2561_GAIN_16X);
					/* Drop the previous conversion results */
					getData(&_b, &_ir);
					/* Set a flag to indicate we've adjusted the gain */
					_agcCheck = true;
				}
				else if ((_b > _hi) && (_tsl2561Gain == TSL2561_GAIN_16X))
				{
					/* Drop gain to 1x and try again */
					setGain(TSL2561_GAIN_1X);
					/* Drop the previous conversion results */
					getData(&_b, &_ir);
					/* Set a flag to indicate we've adjusted the gain */
					_agcCheck = true;
				}
				else
				{
					/* Nothing to look at here, keep moving ....
					 Reading is either valid, or we're already at the chips limits */
					*broadband = _b;
					*ir = _ir;
					valid = true;
				}
			}
			else
			{
				/* If we've already adjusted the gain once, just return the new results.
				 This avoids endless loops where a value is at one extreme pre-gain,
				 and the the other extreme post-gain */
				*broadband = _b;
				*ir = _ir;
				valid = true;
			}
		} while (!valid);
	}
	
	/**************************************************************************/
	/*!
	 @brief  Converts the raw sensor values to the standard SI lux equivalent.
	 @param  broadband The 16-bit sensor reading from the IR+visible light diode.
	 @param  ir The 16-bit sensor reading from the IR-only light diode.
	 @returns The integer Lux value we calcuated.
	 Returns 0 if the sensor is saturated and the values are
	 unreliable, or 65536 if the sensor is saturated.
	 */
	/**************************************************************************/
	/**************************************************************************/
	/*!
	 
	 Returns
	 */
	/**************************************************************************/
	uint32_t calculateLux(uint16_t broadband, uint16_t ir){
		unsigned long chScale;
		unsigned long channel1;
		unsigned long channel0;
		
		/* Make sure the sensor isn't saturated! */
		uint16_t clipThreshold;
		switch (_tsl2561IntegrationTime)
		{
			case TSL2561_INTEGRATIONTIME_13MS:
				clipThreshold = TSL2561_CLIPPING_13MS;
				break;
			case TSL2561_INTEGRATIONTIME_101MS:
				clipThreshold = TSL2561_CLIPPING_101MS;
				break;
			default:
				clipThreshold = TSL2561_CLIPPING_402MS;
				break;
		}
		
		/* Return 65536 lux if the sensor is saturated */
		if ((broadband > clipThreshold) || (ir > clipThreshold))
		{
			return 65536;
		}
		
		/* Get the correct scale depending on the intergration time */
		switch (_tsl2561IntegrationTime)
		{
			case TSL2561_INTEGRATIONTIME_13MS:
				chScale = TSL2561_LUX_CHSCALE_TINT0;
				break;
			case TSL2561_INTEGRATIONTIME_101MS:
				chScale = TSL2561_LUX_CHSCALE_TINT1;
				break;
			default: /* No scaling ... integration time = 402ms */
				chScale = (1 << TSL2561_LUX_CHSCALE);
				break;
		}
		
		/* Scale for gain (1x or 16x) */
		if (!_tsl2561Gain) chScale = chScale << 4;
		
		/* Scale the channel values */
		channel0 = (broadband * chScale) >> TSL2561_LUX_CHSCALE;
		channel1 = (ir * chScale) >> TSL2561_LUX_CHSCALE;
		
		/* Find the ratio of the channel values (Channel1/Channel0) */
		unsigned long ratio1 = 0;
		if (channel0 != 0) ratio1 = (channel1 << (TSL2561_LUX_RATIOSCALE+1)) / channel0;
		
		/* round the ratio value */
		unsigned long ratio = (ratio1 + 1) >> 1;
		
		unsigned int b, m;
		
#ifdef TSL2561_PACKAGE_CS
		if ((ratio >= 0) && (ratio <= TSL2561_LUX_K1C))
		{b=TSL2561_LUX_B1C; m=TSL2561_LUX_M1C;}
		else if (ratio <= TSL2561_LUX_K2C)
		{b=TSL2561_LUX_B2C; m=TSL2561_LUX_M2C;}
		else if (ratio <= TSL2561_LUX_K3C)
		{b=TSL2561_LUX_B3C; m=TSL2561_LUX_M3C;}
		else if (ratio <= TSL2561_LUX_K4C)
		{b=TSL2561_LUX_B4C; m=TSL2561_LUX_M4C;}
		else if (ratio <= TSL2561_LUX_K5C)
		{b=TSL2561_LUX_B5C; m=TSL2561_LUX_M5C;}
		else if (ratio <= TSL2561_LUX_K6C)
		{b=TSL2561_LUX_B6C; m=TSL2561_LUX_M6C;}
		else if (ratio <= TSL2561_LUX_K7C)
		{b=TSL2561_LUX_B7C; m=TSL2561_LUX_M7C;}
		else if (ratio > TSL2561_LUX_K8C)
		{b=TSL2561_LUX_B8C; m=TSL2561_LUX_M8C;}
#else
		if ((ratio >= 0) && (ratio <= TSL2561_LUX_K1T))
		{b=TSL2561_LUX_B1T; m=TSL2561_LUX_M1T;}
		else if (ratio <= TSL2561_LUX_K2T)
		{b=TSL2561_LUX_B2T; m=TSL2561_LUX_M2T;}
		else if (ratio <= TSL2561_LUX_K3T)
		{b=TSL2561_LUX_B3T; m=TSL2561_LUX_M3T;}
		else if (ratio <= TSL2561_LUX_K4T)
		{b=TSL2561_LUX_B4T; m=TSL2561_LUX_M4T;}
		else if (ratio <= TSL2561_LUX_K5T)
		{b=TSL2561_LUX_B5T; m=TSL2561_LUX_M5T;}
		else if (ratio <= TSL2561_LUX_K6T)
		{b=TSL2561_LUX_B6T; m=TSL2561_LUX_M6T;}
		else if (ratio <= TSL2561_LUX_K7T)
		{b=TSL2561_LUX_B7T; m=TSL2561_LUX_M7T;}
		else if (ratio > TSL2561_LUX_K8T)
		{b=TSL2561_LUX_B8T; m=TSL2561_LUX_M8T;}
#endif
		
		unsigned long temp;
		temp = ((channel0 * b) - (channel1 * m));
		
		/* Do not allow negative lux value */
		if (temp < 0) temp = 0;
		
		/* Round lsb (2^(LUX_SCALE-1)) */
		temp += (1 << (TSL2561_LUX_LUXSCALE-1));
		
		/* Strip off fractional portion */
		uint32_t lux = temp >> TSL2561_LUX_LUXSCALE;
		
		/* Signal I2C had no errors */
		return lux;
	}
	
	/* Unified Sensor API Functions */
	
	/**************************************************************************/
	/*!
	 @brief  Gets the most recent sensor event
	 @param  event Pointer to a sensor_event_t type that will be filled
	 with the lux value, timestamp, data type and sensor ID.
	 @returns True if sensor reading is between 0 and 65535 lux,
	 false if sensor is saturated
	 */
	/**************************************************************************/
	
	bool getEvent(sensors_event_t* event){
		uint16_t broadband, ir;
		
		/* Clear the event */
		memset(event, 0, sizeof(sensors_event_t));
		
		event->version   = sizeof(sensors_event_t);
		event->sensor_id = _tsl2561SensorID;
		event->type      = SENSOR_TYPE_LIGHT;
		event->timestamp = (int32_t)ofGetSystemTimeMicros() / 1000;
		
		/* Calculate the actual lux value */
		getLuminosity(&broadband, &ir);
		event->light = calculateLux(broadband, ir);
		
		if (event->light == 65536) {
			return false;
		}
		return true;
	}
	/**************************************************************************/
	/*!
	 @brief  Gets the sensor_t data
	 @param  sensor A pointer to a sensor_t structure that we will fill with
	 details about the TSL2561 and its capabilities
	 */
	/**************************************************************************/
	void getSensor(sensor_t* sensor){
		/* Clear the sensor_t object */
		memset(sensor, 0, sizeof(sensor_t));
		
		/* Insert the sensor name in the fixed length char array */
		strncpy (sensor->name, "TSL2561", sizeof(sensor->name) - 1);
		sensor->name[sizeof(sensor->name)- 1] = 0;
		sensor->version     = 1;
		sensor->sensor_id   = _tsl2561SensorID;
		sensor->type        = SENSOR_TYPE_LIGHT;
		sensor->min_delay   = 0;
		sensor->max_value   = 17000.0;  /* Based on trial and error ... confirm! */
		sensor->min_value   = 1.0;
		sensor->resolution  = 1.0;
	}
	
private:
	std::shared_ptr<I2c> _i2c;
	
	int8_t _addr;
	bool _tsl2561Initialised;
	bool _tsl2561AutoGain;
	tsl2561IntegrationTime_t _tsl2561IntegrationTime;
	tsl2561Gain_t _tsl2561Gain;
	int32_t _tsl2561SensorID;
	/**************************************************************************/
	/*!
	 Enables the device
	 */
	/**************************************************************************/
	void     enable (void){
		/* Enable the device by setting the control bit to 0x03 */
		write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWERON);
	}
	/**************************************************************************/
	/*!
	 Disables the device (putting it in lower power sleep mode)
	 */
	/**************************************************************************/
	void     disable (void){
		/* Turn the device off to save power */
		write8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWEROFF);
	}
	
	/**************************************************************************/
	/*!
	 @brief  Writes a register and an 8 bit value over I2C
	 @param  reg I2C register to write the value to
	 @param  value The 8-bit value we're writing to the register
	 */
	/**************************************************************************/
	void     write8 (uint8_t reg, uint8_t value){
		_i2c->writeByte(reg, value);
		usleep(500);
	}
	
	/**************************************************************************/
	/*!
	 @TODO
	 @brief  Reads an 8 bit value over I2C
	 @param  reg I2C register to read from
	 @returns 8-bit value containing single byte data read
	uint8_t  read8 (uint8_t reg){}
	 */
	/**************************************************************************/


	/**************************************************************************/
	/*!
	 @brief  Reads a 16 bit values over I2C
	 @param  reg I2C register to read from
	 @returns 16-bit value containing 2-byte data read
	 */
	/**************************************************************************/
	uint16_t read16 (uint8_t reg){
		return _i2c->readByte(reg);
	}
	
	void     getData (uint16_t *broadband, uint16_t *ir){
		/* Enable the device by setting the control bit to 0x03 */
		enable();
		
		/* Wait x ms for ADC to complete */
		switch (_tsl2561IntegrationTime)
		{
			case TSL2561_INTEGRATIONTIME_13MS:
				usleep(TSL2561_DELAY_INTTIME_13MS);  // KTOWN: Was 14ms
				break;
			case TSL2561_INTEGRATIONTIME_101MS:
				usleep(TSL2561_DELAY_INTTIME_101MS); // KTOWN: Was 102ms
				break;
			default:
				usleep(TSL2561_DELAY_INTTIME_402MS); // KTOWN: Was 403ms
				break;
		}
		
		/* Reads a two byte value from channel 0 (visible + infrared) */
		*broadband = read16(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN0_LOW);
		
		/* Reads a two byte value from channel 1 (infrared) */
		*ir = read16(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN1_LOW);
		
		/* Turn the device off to save power */
		disable();
	}
};
typedef Adafruit_TSL2561_Unified TSL2561;
