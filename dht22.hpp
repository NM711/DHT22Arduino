#include <Arduino.h>

/*
        Everything is built taking these specs into account:
        1. https://cdn-shop.adafruit.com/datasheets/Digital+humidity+and+temperature+sensor+AM2302.pdf
        2. https://github.com/dvarrel/DHT22/blob/main/extras/DHT22-datasheet.pdf


        ALSO MUST GIVE CREDIT TO THIS REPOSITORY https://github.com/dvarrel/DHT22 AND OTHERS I REFERENCED

        NOTE:
        Something I learned is that there is no seperate input/output pins, rather the 1-Wire protocol allows the DHT22 and
   the microcontroller to share data from a single endpoint (pin), acting as input or output based on turn taking of who
   pulls on the data line and who releases it.

        EXTRA_NOTE:
        I AM LEARNING AS I GO
*/


// TODO REFACTOR THE WAIT/INTERVAL LOOPS.
enum ErrorState {
  NONE,
  // If the interval between reads is over 50microseconds
  PAST_READ_INTERVAL_LIMIT,
  // Above the ranges of 20 - 80, for 1 and 0
  READ_LENGTH_INVALID,
  // sum != checksum
  CHECKSUM_INVALID,
  // 80 Microseconds handshake failed.
  UNAKNOWLEDGED_TRANSMISSION
};

// Each output data signal, or if we wanna think in VM terms, instruction. Is a total of 40 bits.
// -> 16 bits RH (Humidity)
// -> 16 bits Temperature
// -> 8 bit Checksum

enum DataKind {
  HUMIDITY,
  TEMPERATURE,
  TEMPERATURE_MSB,
  CHECKSUM
};

enum TemperatureConversionKind {
  CELSIUS,
  FARENHEIGHT
};

class DHT22Sensor {
  private:
    unsigned pin;
    ErrorState error;
    unsigned long long validSensorData;

    bool handshake() {
      // First we need to setup the "turn taking"
      // NOTE RULE OF THUMB, FOR SINGLE WIRE PROTOCOLS "PULL DOWN" MEANS OUTPUT ON LOW
      // "PULL UP" MEANS INPUT ON HIGH
      // Host (microcontroller) starts the signal by pulling down

      // Send low signal then send high signal
      pinMode(this->pin, OUTPUT);
      digitalWrite(this->pin, LOW);
      // Whole process must take at least 1 - 10 milliseconds, so before pulling up we wil wait 2 ms
      // Now we start receiving or listening for the sensor data by pulling up
      delay(2);
      digitalWrite(this->pin, HIGH);

      // Start receiving sensor input

      pinMode(this->pin, INPUT);
      return this->aknowledgment();
    };

    /*
      Awaits the 80 microsecond low, and the 80 microsecond high to confirm that the sensor aknowledges the transmission.
    */

    bool aknowledgment() {
      unsigned long timerStart, timerEnd, elapsed;
      if (digitalRead(this->pin) != 0) {
        // Give it some time to pull low.
        timerStart = micros();

        while (digitalRead(this->pin) == 1) {
          timerEnd = micros();
          // 40 microseconds
          elapsed = timerEnd - timerStart;
          if (elapsed > 40) {
            return false;
          };
        }
      };

      // 80 Microsecond waits, but ill give it a window to reach above 100
      timerStart = micros();
      while (digitalRead(this->pin) == 0) {
        timerEnd = micros();

        elapsed = timerEnd - timerStart;

        if (elapsed > 100) {
          return false;
        };
      };

      timerStart = micros();
      while (digitalRead(this->pin) == 1) {
        timerEnd = micros();

        elapsed = timerEnd - timerStart;

        if (elapsed > 100) {
          return false;
        };
      };

      return true;
    };

    // 16RH;16TEMP;8CHECKSUM; 40 bits total, but a single extract will never go over 16 bits.

    unsigned short extractKind(unsigned long long data, DataKind kind) {
      switch (kind) {
        case DataKind::HUMIDITY:
          // the only thing im unsure of is whether i need to shift or not?
          // BIN_MASK: 11111111 11111111 00000000 00000000 00000000 => HEX_MASK: 0xFFFF000000
          return (data & 0xFFFF000000ULL) >> 24;
        case DataKind::TEMPERATURE:
          // BIN_MASK: 00000000 00000000 11111111 11111111 00000000 => HEX_MASK: 0x0000FFFF00
          return (data & 0x0000FFFF00ULL) >> 8;
        case DataKind::CHECKSUM:
          // BIN_MASK: 00000000 00000000 00000000 00000000 11111111 => HEX_MASK: 0x00000000FF
          return (data & 0x00000000FFULL);
        case DataKind::TEMPERATURE_MSB:
          // 0x8 = 1000, starting at bit 17
          return (data & 0x0000800000ULL) >> 23;
      };
    };

    /*
      We validate the checksum by comparing the first 32 bits against the last 8 bit checksum
    */
    bool validateChecksum(unsigned long long data) {
      unsigned char checksum = this->extractKind(data, DataKind::CHECKSUM);

      // Shift each byte into a low position, extract that viam ask and add it.
      unsigned char sum = ((data >> 32) & 0xFF) + ((data >> 24) & 0xFF) + ((data >> 16) & 0xFF) + ((data >> 8) & 0xFF);
      return sum == checksum;
    };

    double convertCelsius(unsigned short temperatureBits) {
      return temperatureBits / 10;
    };

    double convertFarenheight(unsigned short temperatureBits) {
      return (this->convertCelsius(temperatureBits) * 1.8) + 32;
    };

  public:
    DHT22Sensor(unsigned dataPin) {
      this->pin = dataPin;
      this->error = ErrorState::NONE;
      this->validSensorData = 0;
    };

    double getTemperature(TemperatureConversionKind kind = TemperatureConversionKind::CELSIUS) {
      // When MSB of temperatures 16 bits is 1, it means we are under 0 degrees Celcius.
      bool isBelowCelsius = this->extractKind(this->validSensorData, DataKind::TEMPERATURE_MSB);
      unsigned short temperature = this->extractKind(this->validSensorData, DataKind::TEMPERATURE);
      if (isBelowCelsius) {
        // 0 out the MSB bit used to check the sign.
        // 0x7FFF = 0111 1111 1111 1111
        // Just update the existing temperature variable
        temperature = (temperature & 0x7FFF);

        return kind == TemperatureConversionKind::FARENHEIGHT ? -(this->convertFarenheight(temperature)) : this->convertCelsius(temperature);
      } else {
        return kind == TemperatureConversionKind::FARENHEIGHT ? this->convertFarenheight(temperature): this->convertCelsius(temperature);
      };
    };

    double getHumidity() {
      return this->extractKind(this->validSensorData, DataKind::HUMIDITY) / 10;
    };

    /*
      Will return any available error states
    */
    ErrorState getErrorState() {
      return this->error;
    };

    // This way we have some way to return the error to its original state without having it go on and on and on, when its no longer the case.
    void resetErrorState() {
     this->error = ErrorState::NONE; 
    }

    /*
            Reads 40 bits from the signal sent from the data pin.
            Essentially we read from the selected digital output ping the sensor is connected to.
    */

    void readDataBits() {
      if (!this->handshake()) {
        this->error = ErrorState::UNAKNOWLEDGED_TRANSMISSION;
        return;
      }
      // 1. A bit is = 0 if the time of the transmission is between 26-28 microseconds
      // 2. A bit is = 1 if the time of the transmission is 70 microseconds
      //
      // We need to get the time and measure it.
      unsigned long long data = 0;
      unsigned char bitsRead = 0;
      // I belive that the sensor transmission remains open till we decide to exit.

      unsigned long long timerStart, timerEnd, elapsed;
      while (bitsRead < 40) {
        timerStart = micros();
        // 50 microsecond stall, but the signal isnt perfect so ill extend the limit to 65
        while (digitalRead(this->pin) == 0) {
          timerEnd = micros();
          elapsed = timerEnd - timerStart;
          if (elapsed > 65) {
            this->error = ErrorState::PAST_READ_INTERVAL_LIMIT;
            // Abort transmission, have caller wait 2 seconds before next call.
            return;
          };
        };

        timerStart = micros();
        // Here is the thing, we cant do comparisons within the loop because we never let the elapsed time actually
        // accumulate a time beyond 1 second. This means an error will always be thrown and the transmission will be aborted,
        // what we must do, is accumulate the elapsed time for however long the pin is on high voltage, then outside we can
        // do our comparison.
        while (digitalRead(this->pin) == 1) {
          timerEnd = micros();
          elapsed = timerEnd - timerStart;
        };

        // 26 - 28 microseconds = 0
        // 70 microseconds = 1
        //
        // Realistically ill add a safety range, as to not get some unecessary off by ones. I dont think the signal will
        // work to perfection each time

        // Down here we will stream the given bit to the data variable

        if (elapsed >= 20 && elapsed < 50) {
          data = (data << 1) | 0;
        } else if (elapsed >= 50 && elapsed <= 100) {
          data = (data << 1) | 1;
        } else {
          this->error = ErrorState::READ_LENGTH_INVALID;
          return;
        };

        bitsRead += 1;
      };

      if (!this->validateChecksum(data)) {
        this->error = ErrorState::CHECKSUM_INVALID;
        return;
      };

      this->validSensorData = data;
    };

};
