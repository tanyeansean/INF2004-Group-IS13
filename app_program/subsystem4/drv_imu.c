/*
 *	drv_imu.c
 *	Subsystem 4 -- GY-511 accelerometer over I2C1 (Buddy 4).
 *
 *	  GY-511 = LSM303D, address 0x1D, on Grove 2: GP2 = SDA, GP3 = SCL.
 *	  100 readings per second, X/Y/Z on, range +/-4 g (8192 per g).
 *
 *	Own register-level driver, like drv_motor.c and drv_track.c.  The
 *	kernel's I2C driver is not used: it only starts I2C0, whose default
 *	pins GP8/GP9 are the motors, and switching it to I2C1 would mean
 *	editing shared kernel files.
 *
 *	The wait loops poll the I2C block.  One read is 8 bytes at 400 kHz,
 *	about 0.2 ms, and any higher-priority task can still run during it.
 *	Each wait has a time limit, so a loose wire returns an error instead
 *	of hanging the task.
 */

#include <sys/machine.h>
#include "subsystem4.h"
#include "drv_imu.h"

/* ---- the sensor ---- */
#define IMU_ADDR		(0x1DU)
#define IMU_CTRL1		(0x20U)
#define IMU_CTRL2		(0x21U)
#define IMU_OUT_X_L		(0x28U)
#define IMU_AUTO_INC		(0x80U)	/* read X, Y, Z in one go */
#define IMU_CTRL1_100HZ_XYZ	(0x67U)
#define IMU_CTRL2_4G		(0x08U)

/* ---- pins ---- */
#define S4_PIN_SDA		(2U)
#define S4_PIN_SCL		(3U)
#define S4_IO_BANK0		(0x40014000U)
#define S4_GPIO_CTRL(pin)	(S4_IO_BANK0 + 0x04U + (8U * (pin)))
#define S4_FUNCSEL_I2C		(3U)
#define S4_PADS_BANK0		(0x4001C000U)
#define S4_PAD(pin)		(S4_PADS_BANK0 + 0x04U + (4U * (pin)))
#define S4_PAD_I2C		((1U << 6) | (1U << 4) | (1U << 3) | (1U << 1))
				/* input on, 4 mA, pull-up, Schmitt trigger */

/* ---- RP2040 I2C1 registers ---- */
#define REG_ALIAS_SET		(0x2000U)
#define REG_ALIAS_CLR		(0x3000U)
#define S4_RESETS_RESET		(0x4000C000U)
#define S4_RESETS_DONE		(0x4000C008U)
#define S4_RESET_I2C1		(1U << 4)
#define S4_TIMERAWL		(0x40054028U)	/* 1 us, started by subsystem 3 */

#define I2C1_BASE_ADDR		(0x40048000U)
#define I2C1_CON		(I2C1_BASE_ADDR + 0x00U)
#define I2C1_TAR		(I2C1_BASE_ADDR + 0x04U)
#define I2C1_DATA_CMD		(I2C1_BASE_ADDR + 0x10U)
#define I2C1_FS_SCL_HCNT	(I2C1_BASE_ADDR + 0x1CU)
#define I2C1_FS_SCL_LCNT	(I2C1_BASE_ADDR + 0x20U)
#define I2C1_RAW_INTR		(I2C1_BASE_ADDR + 0x34U)
#define I2C1_RX_TL		(I2C1_BASE_ADDR + 0x38U)
#define I2C1_TX_TL		(I2C1_BASE_ADDR + 0x3CU)
#define I2C1_CLR_TX_ABRT	(I2C1_BASE_ADDR + 0x54U)
#define I2C1_CLR_STOP_DET	(I2C1_BASE_ADDR + 0x60U)
#define I2C1_ENABLE		(I2C1_BASE_ADDR + 0x6CU)
#define I2C1_RXFLR		(I2C1_BASE_ADDR + 0x78U)
#define I2C1_SDA_HOLD		(I2C1_BASE_ADDR + 0x7CU)
#define I2C1_FS_SPKLEN		(I2C1_BASE_ADDR + 0xA0U)

#define I2C_CON_MASTER		((1U << 8) | (1U << 6) | (1U << 5) | (2U << 1) | (1U << 0))
				/* TX empty ctrl, slave off, restart on, fast, master */
#define I2C_INTR_TX_ABRT	(1U << 6)	/* no answer from the sensor */
#define I2C_INTR_STOP_DET	(1U << 9)	/* transfer finished */
#define I2C_CMD_READ		(1U << 8)
#define I2C_CMD_STOP		(1U << 9)
#define I2C_CMD_RESTART		(1U << 10)

#define I2C_TIMEOUT_US		(2000U)

/* Reset I2C1 and set it up as a 400 kHz master talking to the sensor.
   400 kHz from clk_peri 125 MHz: 313 counts, 126 high + 187 low, the same
   numbers as the kernel's driver (device/i2c/sysdepend/rp2040). */
LOCAL void i2c1_setup(void)
{
	out_w(S4_RESETS_RESET + REG_ALIAS_SET, S4_RESET_I2C1);
	out_w(S4_RESETS_RESET + REG_ALIAS_CLR, S4_RESET_I2C1);
	while ((in_w(S4_RESETS_DONE) & S4_RESET_I2C1) == 0U) {
		;
	}

	out_w(I2C1_ENABLE, 0U);
	out_w(I2C1_CON, I2C_CON_MASTER);
	out_w(I2C1_TAR, IMU_ADDR);
	out_w(I2C1_RX_TL, 0U);
	out_w(I2C1_TX_TL, 0U);
	out_w(I2C1_FS_SCL_HCNT, 126U);
	out_w(I2C1_FS_SCL_LCNT, 187U);
	out_w(I2C1_FS_SPKLEN, 11U);
	out_w(I2C1_SDA_HOLD, 38U);
	out_w(I2C1_ENABLE, 1U);
}

/* Clear what the last transfer left behind. */
LOCAL void i2c1_begin(void)
{
	(void)in_w(I2C1_CLR_TX_ABRT);
	(void)in_w(I2C1_CLR_STOP_DET);
	while (in_w(I2C1_RXFLR) != 0U) {
		(void)in_w(I2C1_DATA_CMD);
	}
}

/* A wait ran out of time: the bus is stuck, so start I2C1 again. */
LOCAL ER i2c1_timed_out(void)
{
	i2c1_setup();
	return E_TMOUT;
}

/* Write one sensor register. */
LOCAL ER imu_write_reg(UW reg, UW val)
{
	UW const start = in_w(S4_TIMERAWL);

	i2c1_begin();
	out_w(I2C1_DATA_CMD, reg);
	out_w(I2C1_DATA_CMD, val | I2C_CMD_STOP);

	for (;;) {
		UW const raw = in_w(I2C1_RAW_INTR);

		if ((raw & I2C_INTR_TX_ABRT) != 0U) {
			return E_IO;
		}
		if ((raw & I2C_INTR_STOP_DET) != 0U) {
			return E_OK;
		}
		if ((in_w(S4_TIMERAWL) - start) > I2C_TIMEOUT_US) {
			return i2c1_timed_out();
		}
	}
}

/* Start I2C1 and the sensor.  An error means the sensor didn't answer. */
EXPORT ER drv_imu_init(void)
{
	ER er;

	out_w(S4_GPIO_CTRL(S4_PIN_SDA), S4_FUNCSEL_I2C);
	out_w(S4_PAD(S4_PIN_SDA), S4_PAD_I2C);
	out_w(S4_GPIO_CTRL(S4_PIN_SCL), S4_FUNCSEL_I2C);
	out_w(S4_PAD(S4_PIN_SCL), S4_PAD_I2C);
	i2c1_setup();

	er = imu_write_reg(IMU_CTRL1, IMU_CTRL1_100HZ_XYZ);
	if (er < E_OK) {
		return er;
	}
	return imu_write_reg(IMU_CTRL2, IMU_CTRL2_4G);
}

/* Read raw X, Y, Z. */
EXPORT ER drv_imu_read(H *p_x, H *p_y, H *p_z)
{
	UW const	start = in_w(S4_TIMERAWL);
	UB		buf[6];
	INT		i;
	INT		got = 0;

	i2c1_begin();
	out_w(I2C1_DATA_CMD, IMU_OUT_X_L | IMU_AUTO_INC);
	out_w(I2C1_DATA_CMD, I2C_CMD_READ | I2C_CMD_RESTART);
	for (i = 1; i < 5; i++) {
		out_w(I2C1_DATA_CMD, I2C_CMD_READ);
	}
	out_w(I2C1_DATA_CMD, I2C_CMD_READ | I2C_CMD_STOP);

	while (got < 6) {
		if ((in_w(I2C1_RAW_INTR) & I2C_INTR_TX_ABRT) != 0U) {
			return E_IO;
		}
		if (in_w(I2C1_RXFLR) != 0U) {
			buf[got] = (UB)(in_w(I2C1_DATA_CMD) & 0xFFU);
			got++;
		} else if ((in_w(S4_TIMERAWL) - start) > I2C_TIMEOUT_US) {
			return i2c1_timed_out();
		}
	}

	*p_x = (H)(((UH)buf[1] << 8) | buf[0]);
	*p_y = (H)(((UH)buf[3] << 8) | buf[2]);
	*p_z = (H)(((UH)buf[5] << 8) | buf[4]);
	return E_OK;
}
