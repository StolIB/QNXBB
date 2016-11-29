/*
 * $QNXLicenseC:
 * Copyright 2015, QNX Software Systems.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"). You
 * may not reproduce, modify or distribute this software except in
 * compliance with the License. You may obtain a copy of the License
 * at: http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" basis,
 * WITHOUT WARRANTIES OF ANY KIND, either express or implied.
 *
 * This file may contain contributions from others, either as
 * contributors under the License or as licensors under other terms.
 * Please review this entire file for other proprietary rights or license
 * notices, as well as the QNX Development Suite License Guide at
 * http://licensing.qnx.com/license-guide/ for other information.
 * $
 */

#include "ipl.h"
#include "private/omap_sdhc.h"
#include <hw/inout.h>
#include "private/omap_timer.h"
#include <stdlib.h>

#ifdef MMC_DUMP
void omap_dump(sdmmc_t *sdmmc)
{
	sdmmc_hc_t	*hc = sdmmc->hc;
	unsigned	 base = hc->sdmmc_pbase;
	ser_putstr("\nomap_dump\n");

	ser_putstr(" HL_REV "); ser_puthex(in32( base + OMAP_MMCHS_HL_REV ));
	ser_putstr(" HWINFO "); ser_puthex(in32( base + OMAP_MMCHS_HL_HWINFO ));
	ser_putstr(" HL_SYSCONFIG ");ser_puthex(in32( base + OMAP_MMCHS_HL_SYSCONFIG ));
	ser_putstr(" SYSCONFIG ");ser_puthex(in32( base + OMAP_MMCHS_SYSCONFIG ));
	ser_putstr(" SYSSTATUS "); ser_puthex(in32( base + OMAP_MMCHS_SYSSTATUS ));
	ser_putstr(" REV "); ser_puthex(in32( base + OMAP_MMCHS_REV ));
	ser_putstr(" CAPA "); ser_puthex(in32( base + OMAP_MMCHS_CAPA ));
	ser_putstr(" CAPA2 "); ser_puthex(in32( base + OMAP_MMCHS_CAPA2 ));
	ser_putstr(" MCCAP "); ser_puthex(in32( base + OMAP_MMCHS_CUR_CAPA));
	ser_putstr(" SYSTEST "); ser_puthex(in32( base + OMAP_MMCHS_SYSTEST ));
	ser_putstr(" FE "); ser_puthex(in32( base + OMAP_MMCHS_FE ));
	ser_putstr("\n");

	ser_putstr(" CSRE "); ser_puthex(in32( base + OMAP_MMCHS_CSRE ));
	ser_putstr(" CON "); ser_puthex(in32( base + OMAP_MMCHS_CON ));
	ser_putstr(" PWCNT "); ser_puthex(in32( base + OMAP_MMCHS_PWCNT ));
	ser_putstr(" BLK "); ser_puthex(in32( base + OMAP_MMCHS_BLK ));
	ser_putstr(" ARG "); ser_puthex(in32( base + OMAP_MMCHS_ARG ));
	ser_putstr(" CMD "); ser_puthex(in32( base + OMAP_MMCHS_CMD ));
	ser_putstr(" RSP10 "); ser_puthex(in32( base + OMAP_MMCHS_RSP10 ));
	ser_putstr(" RSP32 "); ser_puthex(in32( base + OMAP_MMCHS_RSP32 ));
	ser_putstr(" RSP54 "); ser_puthex(in32( base + OMAP_MMCHS_RSP54 ));
	ser_putstr(" RSP76 "); ser_puthex(in32( base + OMAP_MMCHS_RSP76) );
	ser_putstr("\n");

	ser_putstr(" PSTATE "); ser_puthex(in32( base + OMAP_MMCHS_PSTATE ));
	ser_putstr(" HCTL "); ser_puthex(in32( base + OMAP_MMCHS_HCTL ));
	ser_putstr(" SYSCTL "); ser_puthex(in32( base + OMAP_MMCHS_SYSCTL ));
	ser_putstr(" STAT "); ser_puthex(in32( base + OMAP_MMCHS_STAT ));
	ser_putstr(" IE "); ser_puthex(in32( base + OMAP_MMCHS_IE ));
	ser_putstr(" ISE "); ser_puthex(in32( base + OMAP_MMCHS_ISE ));
	ser_putstr(" AC12 "); ser_puthex(in32( base + OMAP_MMCHS_AC12 ));
	ser_putstr(" ADMAES "); ser_puthex(in32( base + OMAP_MMCHS_ADMAES ));
	ser_putstr(" ADMASAL "); ser_puthex(in32( base + OMAP_MMCHS_ADMASAL ));
	ser_putstr("\n");
}
#endif

int omap_waitmask( sdmmc_t *sdmmc, unsigned reg, unsigned mask, unsigned val, unsigned usec )
{
	sdmmc_hc_t	*hc = sdmmc->hc;
	unsigned	base = hc->sdmmc_pbase;
	unsigned	cnt;
	int			stat = SDMMC_ERROR;
	int			rval = 0;
	unsigned	iter;

	/* fast poll for 1ms - 1us intervals */
	for( cnt = min( usec, 1000 ); cnt; cnt-- ) {
		if( ( ( rval = in32( base + reg ) ) & mask ) == val ) {
			stat = SDMMC_OK;
			break;
		}
		omap_nano_delay(1000);
	}

	if( usec > 1000 && stat ) {
		iter = usec / 1000L;
		for( cnt = iter; cnt; cnt-- ) {
			if( ( ( rval = in32( base + reg ) ) & mask ) == val ) {
				stat = SDMMC_OK;
				break;
			}
			omap_usec_delay( 1 );
		}
	}

	if( !cnt ) {
		ser_putstr("omap_waitmask reg 0x");ser_puthex(reg);
		ser_putstr(" mask 0x");ser_puthex(mask);
		ser_putstr(" val 0x");ser_puthex(val);
		ser_putstr(" rval 0x");ser_puthex(rval);
		ser_putstr(" stat 0x");ser_puthex(stat); ser_putstr("\n" );
	}

	return( stat );
}

static int omap_mmchs_softreset(sdmmc_t *sdmmc)
{
	sdmmc_hc_t	*hc = sdmmc->hc;
	unsigned	base = hc->sdmmc_pbase;
	int			status;
	int			sysconfig;

	sysconfig = in32( base + OMAP_MMCHS_SYSCONFIG );

	if( sysconfig & SYSCONFIG_AUTOIDLE ) {
		out32( base + OMAP_MMCHS_SYSCONFIG, sysconfig & ~SYSCONFIG_AUTOIDLE );
	}
	out32( base + OMAP_MMCHS_SYSCONFIG, in32( base + OMAP_MMCHS_SYSCONFIG ) | SYSCONFIG_SOFTRESET );
	status = omap_waitmask( sdmmc, OMAP_MMCHS_SYSSTATUS, SYSSTATUS_RESETDONE, SYSSTATUS_RESETDONE, 10000 );

	if( sysconfig & SYSCONFIG_AUTOIDLE ){
		out32( base + OMAP_MMCHS_SYSCONFIG, sysconfig );
	}
	return status;
}

int omap_reset( sdmmc_t *sdmmc, unsigned rst )
{
	unsigned base = sdmmc->hc->sdmmc_pbase;
	int	stat;

	/* wait up to 100 ms for reset to complete */
	out32( base + OMAP_MMCHS_SYSCTL, in32( base + OMAP_MMCHS_SYSCTL ) | rst );
	/* on OMAP54XX the reset bit is cleared before we can test it, therefore this call will always time out. */
	if ( sdmmc->hc->did < OMAP_DID_54XX ) {
		stat = omap_waitmask(sdmmc, OMAP_MMCHS_SYSCTL, rst, rst, 10000);
	}

	stat = omap_waitmask(sdmmc, OMAP_MMCHS_SYSCTL, rst, 0, 100000 );

	if( stat ) {
		ser_putstr("omap_reset timeout\n");
	}

	return( stat );
}

void omap_pwr(sdmmc_t *sdmmc, int vdd){
	sdmmc_hc_t	*hc = sdmmc->hc;
	unsigned	base = hc->sdmmc_pbase;
	unsigned	con;
	unsigned	capa;
	unsigned	hctl;

	out32( base + OMAP_MMCHS_IE, 0 );
	out32( base + OMAP_MMCHS_ISE, 0 );

	if(!vdd) {
		 (void)omap_mmchs_softreset(sdmmc );
		omap_reset(sdmmc, SYSCTL_SRA | SYSCTL_SRC | SYSCTL_SRD );

		hctl	= 0;
		con		= CON_DEBOUNCE_8MS | CON_MNS;
		hctl |= HCTL_DMAS_ADMA2;

		out32( base + OMAP_MMCHS_CON, con );
		capa = in32( base + OMAP_MMCHS_CAPA );

		capa |= CAPA_VS3V3 | CAPA_VS3V0 | CAPA_VS1V8;
		out32( base + OMAP_MMCHS_CAPA, capa );
		out32( base + OMAP_MMCHS_HCTL, hctl );

	}
	else {
		hctl = in32( base + OMAP_MMCHS_HCTL ) & ~HCTL_SDBP;
		out32( base + OMAP_MMCHS_HCTL, hctl );
		hctl = in32( base + OMAP_MMCHS_HCTL ) & ~HCTL_SDVS_MSK;
		out32( base + OMAP_MMCHS_HCTL, hctl );

		hctl |= ( ( vdd <= OCR_VDD_17_195 ) ? HCTL_SDVS1V8 : HCTL_SDVS3V0 );
		out32( base + OMAP_MMCHS_HCTL, hctl );
		out32( base + OMAP_MMCHS_HCTL, hctl | HCTL_SDBP );
		omap_waitmask( sdmmc, OMAP_MMCHS_HCTL, HCTL_SDBP, HCTL_SDBP, 10000 );

		out32( base + OMAP_MMCHS_ISE, INTR_ALL );
	}
}

static int omap_preset( sdmmc_t *sdmmc, int enable )
{
	sdmmc_hc_t	*hc = sdmmc->hc;
	unsigned	base = hc->sdmmc_pbase;
	unsigned	hctl2;

	if( hc->version < REV_SREV_V3 ) {
		return(SDMMC_OK);
	}

	hctl2	= in32( base + OMAP_MMCHS_HCTL2 ) & ~HCTL2_PRESET_VAL;
	hctl2	|= ( ( enable == 1 ) ? HCTL2_PRESET_VAL : 0 );
	out32( base + OMAP_MMCHS_HCTL2, hctl2 );

	return(SDMMC_OK);
}

int omap_signal_voltage( sdmmc_t *sdmmc, int signal_voltage )
{
	sdmmc_hc_t	*hc = sdmmc->hc;
	unsigned	base = hc->sdmmc_pbase;
	unsigned	hctl;
	unsigned	con;
	int			status = 0;

	con		= in32( base + OMAP_MMCHS_CON );
	hctl	= in32( base + OMAP_MMCHS_HCTL ) & ~HCTL_SDVS_MSK;

	if( signal_voltage == SIGNAL_VOLTAGE_1_8 ) {
		omap_usec_delay(10);
		/* force ADPIDLE mode to active state */
		out32( base + OMAP_MMCHS_CON, con | CON_PADEN );

		if( ( hc->did == OMAP_DID_54XX ) || ( hc->did == OMAP_DID_54XX_ES2 ) || ( hc->did == OMAP_DID_DRA7XX ) ||
			( status = omap_waitmask( sdmmc, OMAP_MMCHS_PSTATE, PSTATE_CLEV | PSTATE_DLEV_MSK, 0, MMCHS_PWR_SWITCH_TIMEOUT ) ) == SDMMC_OK ) {

			out32( base + OMAP_MMCHS_HCTL, hctl | HCTL_SDVS1V8 );

			if( hc->version >= REV_SREV_V3 ) out32( base + OMAP_MMCHS_HCTL2, in32( base + OMAP_MMCHS_HCTL2 ) | HCTL2_SIG_1_8V );

			out32( base + OMAP_MMCHS_CON, in32( base + OMAP_MMCHS_CON ) | CON_CLKEXTFREE );

			 /* wait for CLEV = 0x1, DLEV = 0xf */
			status = omap_waitmask( sdmmc, OMAP_MMCHS_PSTATE, PSTATE_CLEV | PSTATE_DLEV_MSK, PSTATE_CLEV | PSTATE_DLEV_MSK, MMCHS_PWR_SWITCH_TIMEOUT );
		}
		/* cut off external card clock, remove forcing from ADPIDLE pin */
		con &= ~( CON_CLKEXTFREE | CON_PADEN );
		out32( base + OMAP_MMCHS_CON, con );

	} else if ( signal_voltage == SIGNAL_VOLTAGE_3_3 ) {
		out32( base + OMAP_MMCHS_HCTL, hctl | HCTL_SDVS3V0 );
		if( hc->version >= REV_SREV_V3 ) out32( base + OMAP_MMCHS_HCTL2, in32( base + OMAP_MMCHS_HCTL2 ) & ~HCTL2_SIG_1_8V );
	} else {
		return(SDMMC_ERROR);
	}

	return( status );
}

#define TUNING_4BIT_BLK_SIZE		64
#define TUNING_8BIT_BLK_SIZE		128
#define MMCHS_TUNING_RETRIES		40
#define MMCHS_TUNING_TIMEOUT		150

static inline void *memset(void *p, int c, unsigned int size)
{
	unsigned char *pp = (unsigned char *)p;

	while (size-- > 0)
		*pp++ = (unsigned char)c;
	return p;
}

static inline int memcmp(unsigned char * p1, unsigned char * p2, int len)
{
	int i;
	for(i=0; i<len; i++)
	{
		if(*(p1 +i ) != *(p1 +i )) return(1);
	}
	return(0);
}

/* tuning block pattern for 4 bit mode */
unsigned char	sdio_tbp_4bit[] = {
	0xff, 0x0f, 0xff, 0x00, 0xff, 0xcc, 0xc3, 0xcc,
	0xc3, 0x3c, 0xcc, 0xff, 0xfe, 0xff, 0xfe, 0xef,
	0xff, 0xdf, 0xff, 0xdd, 0xff, 0xfb, 0xff, 0xfb,
	0xbf, 0xff, 0x7f, 0xff, 0x77, 0xf7, 0xbd, 0xef,
	0xff, 0xf0, 0xff, 0xf0, 0x0f, 0xfc, 0xcc, 0x3c,
	0xcc, 0x33, 0xcc, 0xcf, 0xff, 0xef, 0xff, 0xee,
	0xff, 0xfd, 0xff, 0xfd, 0xdf, 0xff, 0xbf, 0xff,
	0xbb, 0xff, 0xf7, 0xff, 0xf7, 0x7f, 0x7b, 0xde,
};

/* tuning block pattern for 8 bit mode */
unsigned char	sdio_tbp_8bit[] = {
	0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00,
	0xff, 0xff, 0xcc, 0xcc, 0xcc, 0x33, 0xcc, 0xcc,
	0xcc, 0x33, 0x33, 0xcc, 0xcc, 0xcc, 0xff, 0xff,
	0xff, 0xee, 0xff, 0xff, 0xff, 0xee, 0xee, 0xff,
	0xff, 0xff, 0xdd, 0xff, 0xff, 0xff, 0xdd, 0xdd,
	0xff, 0xff, 0xff, 0xbb, 0xff, 0xff, 0xff, 0xbb,
	0xbb, 0xff, 0xff, 0xff, 0x77, 0xff, 0xff, 0xff,
	0x77, 0x77, 0xff, 0x77, 0xbb, 0xdd, 0xee, 0xff,
	0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00,
	0x00, 0xff, 0xff, 0xcc, 0xcc, 0xcc, 0x33, 0xcc,
	0xcc, 0xcc, 0x33, 0x33, 0xcc, 0xcc, 0xcc, 0xff,
	0xff, 0xff, 0xee, 0xff, 0xff, 0xff, 0xee, 0xee,
	0xff, 0xff, 0xff, 0xdd, 0xff, 0xff, 0xff, 0xdd,
	0xdd, 0xff, 0xff, 0xff, 0xbb, 0xff, 0xff, 0xff,
	0xbb, 0xbb, 0xff, 0xff, 0xff, 0x77, 0xff, 0xff,
	0xff, 0x77, 0x77, 0xff, 0x77, 0xbb, 0xdd, 0xee,
};

/* Set DLL phase delay for HS tuning */
static int omap_set_dll(sdmmc_t *sdmmc, int delay)
{
	unsigned		dll;
	sdmmc_hc_t		*hc = sdmmc->hc;
	unsigned		base = hc->sdmmc_pbase;

	dll = ( in32( base + OMAP_MMCHS_DLL ) & ~( DLL_SLAVE_RATIO_MSK | DLL_CALIB ) );
	dll |= ( delay << DLL_SLAVE_RATIO_SHIFT );
	out32( base + OMAP_MMCHS_DLL, dll);
	dll |= DLL_CALIB;
	out32( base + OMAP_MMCHS_DLL, dll);
	/* Flush last write */
	dll = in32( base + OMAP_MMCHS_DLL );

	return (SDMMC_OK);
}

int omap_tune(sdmmc_t *sdmmc, int op)
{
	sdmmc_hc_t	*hc = sdmmc->hc;
	unsigned	base = hc->sdmmc_pbase;
	unsigned	hctl2;
	unsigned	pdelay = 0;
	int			tlc;
	int			tlen;
	int			status;
	int			index = 0xFF;
	int			imax = 0;
	int			wlen = 0;
	int			wmax = 0;
	int			pm = 0;
	unsigned	rsp[4];
	unsigned char td[128];

	tlen = TUNING_8BIT_BLK_SIZE;;

	if( hc->version < REV_SREV_V3 ) {
		return(0 );
	}

	hctl2 = in32( base + OMAP_MMCHS_HCTL2 );
	hctl2 &= ~HCTL2_MODE_MSK;
	hctl2 |= HCTL2_MODE_SDR104;
	out32( base + OMAP_MMCHS_HCTL2, hctl2 );

	/* return if not HS200 or SDR104, and not SDR50 that requires tuning */
	if( ( HCTL2_MODE( hctl2 ) != HCTL2_MODE_SDR104 ) &&
		( HCTL2_MODE( hctl2 ) != HCTL2_MODE_HS200 ) &&
		( ( HCTL2_MODE( hctl2 ) == HCTL2_MODE_SDR50 ) ) ) {
		return(SDMMC_OK);
	}

	/* start the tuning procedure by setting MMCHS_AC12[22] ET to 0x1 */
	hctl2 |= HCTL2_EXEC_TUNING;
	out32( base + OMAP_MMCHS_HCTL2, hctl2 );

	/* it is important to increment tlc to be able to calculate correct phase delay */
	for( tlc = 0; ( tlc < MMCHS_TUNING_RETRIES ); tlc++) {
		/* clear tuning data buffer to avoid comparing old data after unsuccessful transfer */
		memset(td, 0, tlen);

		if (sdmmc->verbose > SDMMC_VERBOSE_LVL_0) {
			ser_putstr(" tuning tlc ");ser_putdec(tlc);
			ser_putstr(" MMCHS_DLL 0x");ser_puthex(in32(base + OMAP_MMCHS_DLL));
			ser_putstr("\n");
		}
		/* check whether the DLL is locked */
		if ( ( status = omap_waitmask(sdmmc , OMAP_MMCHS_DLL, DLL_LOCK, DLL_LOCK, MMCHS_TUNING_TIMEOUT) ) != SDMMC_OK) {
			ser_putstr("OMAP_MMCHS_DLL lock timeout 0x");ser_puthex(status); ser_putstr("\n" );
			break;
		}

		CMD_CREATE (sdmmc->cmd, MMC_SEND_TUNING_BLOCK, 0, rsp, TUNING_8BIT_BLK_SIZE, 1, td);
		if( 0 != (status = hc->pio_read(sdmmc, td, TUNING_8BIT_BLK_SIZE))) {
			if (sdmmc->verbose > SDMMC_VERBOSE_LVL_0) ser_putstr("tuning omap_pio_read error\n");
		}
		/* determine largest timing window where data transfer is working */
		if ((memcmp(td, sdio_tbp_8bit, tlen))) {
			 pm = 0;
		} else {
			if ( pm == 0 ) {
			/* new window with successful transmission */
				index = tlc;
				wlen = 1;
			} else {
				wlen++;
			}
			pm = 1;
			if ( wlen > wmax ) {
				imax = index;
				wmax = wlen;
			}
		}
		/* check if tuning is finished */
		if( !( ( hctl2 = in32( base + OMAP_MMCHS_HCTL2 ) ) & HCTL2_EXEC_TUNING ) ) {
			if((tlc < MMCHS_TUNING_RETRIES) && status ){
				hctl2 |= HCTL2_EXEC_TUNING;
				out32( base + OMAP_MMCHS_HCTL2, hctl2 );
				continue;
			} else {
				break;
			}
		}
	}

	if( !status && ( hctl2 & HCTL2_TUNED_CLK) ) {
		/* tuning successful, set phase delay to the middle of the window */
		pdelay = 2 * (imax + (wmax >> 1));
		/* reset data and command line before setting phase delay, according to OMAP5432 ES2.0 spec. */
		omap_reset( sdmmc, SYSCTL_SRD );
		omap_reset( sdmmc, SYSCTL_SRC );
		omap_set_dll(sdmmc, pdelay);
		if (sdmmc->verbose > SDMMC_VERBOSE_LVL_0) {
			ser_putstr(" tuning successful. Set dll 0x");ser_puthex(pdelay); ser_putstr("\n" );
		}
	} else {
		/* tuning failed */
		hctl2 &= ~( HCTL2_TUNED_CLK | HCTL2_EXEC_TUNING );
		out32( base + OMAP_MMCHS_HCTL2, hctl2 );
		ser_putstr("tuning failed. status 0x");ser_puthex(status); ser_putstr("\n");
		status = SDMMC_ERROR;
	}
	return( status );
}


int omap_init_ctrl_v18(sdmmc_t *sdmmc)
{
	sdmmc_hc_t	*hc = sdmmc->hc;
	unsigned	base = hc->sdmmc_pbase;

	sdmmc->ocr = OCR_VDD_17_195;

	/* Disable All interrupts */
	out32(base + OMAP_MMCHS_IE, 0);

	omap_pwr(sdmmc, OCR_VDD_32_33);
	omap_pwr(sdmmc, 0);
	omap_pwr(sdmmc, OCR_VDD_17_195);

	hc->set_clk(sdmmc, 400);
	hc->set_bus_width(sdmmc, 1);
	omap_preset(sdmmc, 0);

	omap_signal_voltage(sdmmc, SIGNAL_VOLTAGE_1_8);

	out32(base + OMAP_MMCHS_ISE, INTR_ALL);
	out32(base + OMAP_MMCHS_IE, INTR_ALL);

	return SDMMC_OK;
}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/6.6.0/trunk/hardware/ipl/lib/sdmmc/omap_uhs.c $ $Rev: 790821 $")
#endif
