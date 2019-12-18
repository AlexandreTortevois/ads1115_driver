/**
 * © Alexandre TORTEVOIS
 * An Easier ADS111x Driver uses the i2c protocol with the Linux IOCTL interface
 * Based on TI ADS1115 datasheet for Adafruit 1085
 * http://www.ti.com/lit/ds/symlink/ads1115.pdf
 * https://www.adafruit.com/product/1085
 *
 *  
 * Load:   dtoverlay=ads1115,<param>[=<val>]
 * Params: addr                    I2C bus address of device. Set based on how the
 *                                 addr pin is wired. (default=0x48 assumes addr
 *                                 is pulled to GND)
 *         cha_enable              Enable virtual channel a.
 *         cha_cfg                 Set the configuration for virtual channel a.
 *                                 (default=4 configures this channel for the
 *                                 voltage at A0 with respect to GND)
 *         cha_datarate            Set the datarate (samples/sec) for this channel.
 *                                 (default=7 sets 860 sps)
 *        cha_gain                Set the gain of the Programmable Gain
 *                                 Amplifier for this channel. (Default 1 sets the
 *                                 full scale of the channel to 4.096 Volts)
 *
 *
 * Enable i2c with raspi-config
 * Install the i2c tools : apt-get install i2c-tools
 * Install the ADC :
 * - Launch i2cdetect -y 1 to get the Device Address
 * - Edit /boot/config.txt and add dtoverlay=ads1115,dtparam=cha_enable,dtparam=chb_enable,dtparam=chc_enable,dtparam=chd_enable at the end
 * - Reboot
 * - Launch i2cdetect -y 1, now Address must be UU
 *
 */

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <zconf.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>

#define ADC1115_A0 4
#define ADC1115_A1 5
#define ADC1115_A2 6
#define ADC1115_A3 7

/** Defined Full-Scale Range. It's configured by bits PGA[2:0] in the Config register (see Table 8 p.28) */
const float FSR = 4.096;

/** Define the scale in Volts Per Step, used for the conversion in Volt */
const float VPS = FSR / 32768.0;

/** I2C path */
const char *i2c_path = "/dev/i2c-1";

/** I2C Device Address (see Table 4 p.23) */
const unsigned char i2d_addr = 0x48;

/** I2C Device File Descriptor */
int i2c_fd = -1;

/** Compute MSB of Config Register (see Table 8 p.28-29) */
unsigned char config_register_msb(unsigned char mode, unsigned char pga, unsigned char mux, unsigned char os) {
    unsigned char result;
    result = mode;
    result |= pga << 1;
    result |= mux << 4; // Here is configured the channel to read
    result |= os << 7;
    return result;
}

/** Compute LSB of Config Register (see Table 8 p.28-29) */
unsigned char config_register_lsb(unsigned char comp_que, unsigned char comp_lat, unsigned char comp_pol, unsigned char comp_mode, unsigned char dr) {
    unsigned char result;
    result = comp_que;
    result |= comp_lat << 2;
    result |= comp_pol << 3;
    result |= comp_mode << 4;
    result |= dr << 5;
    return result;
}

/** Send a configuration message before to get the value for the channel */
static int i2c_send_config(unsigned char channel) {
    int rc = 0;
    struct i2c_msg i2c_msgs[1];
    struct i2c_rdwr_ioctl_data i2c_msgset;
    unsigned char config_buf[3];

    config_buf[0] = 1; // Config register mode
    config_buf[1] = config_register_msb(0b1, 0b001, channel, 0b1);
    config_buf[2] = config_register_lsb(0b11, 0b0, 0b0, 0b0, 0b100);

    i2c_msgs[0].addr = i2d_addr;
    i2c_msgs[0].flags = 0;
    i2c_msgs[0].len = sizeof(config_buf);
    i2c_msgs[0].buf = config_buf;

    i2c_msgset.msgs = i2c_msgs;
    i2c_msgset.nmsgs = 1;

    if ((rc = ioctl(i2c_fd, I2C_RDWR, &i2c_msgset)) < 0) {
        printf("Unable to send the configuration message for channel %u : %s\n", channel, strerror(errno));
        goto __error;
    }

    __error:
    return rc;
}

/** Send a message to Get the value. Compute value between 0 and 32767 */
int i2c_get_value(int channel, int *value) {
    int rc = 0;
    struct i2c_msg i2c_msgs[2];
    struct i2c_rdwr_ioctl_data i2c_msgset;
    unsigned char data[2];

    if ((rc = i2c_send_config(channel)) < 0) {
        goto __error;
    }

    usleep(10000);

    i2c_msgs[0].addr = i2d_addr;
    i2c_msgs[0].flags = 0;
    i2c_msgs[0].len = 1;
    i2c_msgs[0].buf = "\x00";

    i2c_msgs[1].addr = i2d_addr;
    i2c_msgs[1].flags = I2C_M_RD;
    i2c_msgs[1].len = 2;
    i2c_msgs[1].buf = (char *) &data;

    i2c_msgset.msgs = i2c_msgs;
    i2c_msgset.nmsgs = 2;

    if ((rc=ioctl(i2c_fd, I2C_RDWR, &i2c_msgset)) < 0) {
        printf("Unable to read channel %u : %s\n", channel, strerror(errno));
        goto __error;
    };

    // Compute the 2 bits, 0:MSB, 1:LSB (see p.35)
    *value = data[0] << 8 | data[1];

    // Hack, to be sure to haven't value less than 0 !
    if (*value > 0x7FFF) *value = 0;

    __error:
    return rc;
}

int main(void) {
    int value;

    // Get a file descriptor for the device
    if ((i2c_fd = open(i2c_path, O_RDWR)) < 0) {
        printf("Error: Couldn't open device! %d\n", i2c_fd);
        exit(EXIT_FAILURE);
    }

    // Connect to ADS111x as i2c Slave (see p.23)
    if (ioctl(i2c_fd, I2C_SLAVE_FORCE, i2d_addr) < 0) {
        printf("Error: Couldn't find device on i2d_addr!\n");
        exit(EXIT_FAILURE);
    }

    for( int i = 0 ; i < 100 ; i++)  {
        printf("%3d", i);
        if (i2c_get_value(ADC1115_A0, &value) >= 0) {
            printf(" | A0 HEX: 0x%04x DEC: %5d Mesure: %4.3f V", value, value, ((float) value * VPS));
        }
        if (i2c_get_value(ADC1115_A1, &value) >= 0) {
            printf(" | A1 HEX: 0x%04x DEC: %5d Mesure: %4.3f V", value, value, ((float) value * VPS));
        }
        if (i2c_get_value(ADC1115_A2, &value) >= 0) {
            printf(" | A2 HEX: 0x%04x DEC: %5d Mesure: %4.3f V", value, value, ((float) value * VPS));
        }
        if (i2c_get_value(ADC1115_A3, &value) >= 0) {
            printf(" | A3 HEX: 0x%04x DEC: %5d Mesure: %4.3f V", value, value, ((float) value * VPS));
        }
        printf("\n");
        usleep(250000);
    }

    EXIT_SUCCESS;
}