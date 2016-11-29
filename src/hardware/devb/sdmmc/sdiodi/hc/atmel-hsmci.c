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

#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <malloc.h>
#include <unistd.h>
#include <hw/inout.h>
#include <sys/mman.h>

#include <internal.h>

#ifdef SDIO_HC_ATMEL_HSMCI

#include <atmel-hsmci.h>
#include <drvr/hwinfo.h>

static int atmel_hsmci_pm(sdio_hc_t *hc, int action);
static int atmel_hsmci_cmd(sdio_hc_t *hc, sdio_cmd_t *cmd);
static int atmel_hsmci_abort(sdio_hc_t *hc, sdio_cmd_t *cmd);
static int atmel_hsmci_event(sdio_hc_t *hc, sdio_event_t *ev);
static int atmel_hsmci_cd(sdio_hc_t *hc);
static int atmel_hsmci_pwr(sdio_hc_t *hc, int vdd);
static int atmel_hsmci_clk(sdio_hc_t *hc, int clk);
static int atmel_hsmci_mode(sdio_hc_t *hc, int bus_mode);
static int atmel_hsmci_bus_width(sdio_hc_t *hc, int width);
static int atmel_hsmci_timing(sdio_hc_t *hc, int timing);
static int atmel_hsmci_signal_voltage(sdio_hc_t *hc, int signal_voltage);

static sdio_hc_entry_t atmel_hsmci_hc_entry =   {
    .nentries        = 16,
    .dinit           = atmel_hsmci_dinit,
    .pm              = atmel_hsmci_pm,
    .cmd             = atmel_hsmci_cmd,
    .abort           = atmel_hsmci_abort,
    .event           = atmel_hsmci_event,
    .cd              = atmel_hsmci_cd,
    .pwr             = atmel_hsmci_pwr,
    .clk             = atmel_hsmci_clk,
    .bus_mode        = atmel_hsmci_mode,
    .bus_width       = atmel_hsmci_bus_width,
    .timing          = atmel_hsmci_timing,
    .signal_voltage  = atmel_hsmci_signal_voltage,
    .drv_type        = NULL,
    .driver_strength = NULL,
    .tune            = NULL,
    .preset          = NULL,
};


/* Enable HSMCI */
static inline void atmel_hsmci_enable(sdio_hc_t *hc) {
    atmel_hc_hsmci_t *hsmci = (atmel_hc_hsmci_t*)hc->cs_hdl;

    out32(hsmci->base + ATMEL_HSMCI_CR, ATMEL_HSMCI_CR_MCIEN);
}

/* Disable HSMCI */
static inline void atmel_hsmci_disable(sdio_hc_t *hc) {
    atmel_hc_hsmci_t *hsmci = (atmel_hc_hsmci_t*)hc->cs_hdl;

    out32(hsmci->base + ATMEL_HSMCI_CR, ATMEL_HSMCI_CR_MCIDIS);
}

/* Enable HSMCI interrupt */
static inline void atmel_hsmci_interrupt_enable(sdio_hc_t *hc, uint32_t interrupt) {
    atmel_hc_hsmci_t *hsmci = (atmel_hc_hsmci_t*)hc->cs_hdl;

    out32(hsmci->base + ATMEL_HSMCI_IER, interrupt);
}

/* Disable HSMCI interrupt */
static inline void atmel_hsmci_interrupt_disable(sdio_hc_t *hc, uint32_t interrupt) {
    atmel_hc_hsmci_t *hsmci = (atmel_hc_hsmci_t*)hc->cs_hdl;

    out32(hsmci->base + ATMEL_HSMCI_IDR, interrupt);
}

/* Perform software reset of HSMCI and setup defaults */
static void atmel_hsmci_swreset(sdio_hc_t *hc) {
    atmel_hc_hsmci_t *hsmci = (atmel_hc_hsmci_t*)hc->cs_hdl;

    /* Request software reset */
    out32(hsmci->base + ATMEL_HSMCI_CR, ATMEL_HSMCI_CR_SWRST);

    atmel_hsmci_disable(hc);

    /* Configure mode register */
    uint32_t mr = 0;
    mr |= ATMEL_HSMCI_MR_PWSDIV_MAX;
    mr |= ATMEL_HSMCI_MR_RDPROOF | ATMEL_HSMCI_MR_WRPROOF;
    out32(hsmci->base + ATMEL_HSMCI_MR, mr);

    /* Set maximum data timeout */
    out32(hsmci->base + ATMEL_HSMCI_DTOR, ATMEL_HSMCI_DTOR_MAX);

    /* Enable automatic power saving mode */
    out32(hsmci->base + ATMEL_HSMCI_CR, ATMEL_HSMCI_CR_PWSEN);

    /* DONE! (HSMCI is disabled) */
}

static int atmel_hsmci_pm(sdio_hc_t *hc, int action) {
    switch(action) {
        case PM_IDLE:
        case PM_ACTIVE:
            /* Note: we have power saving enabled, so idle is managed by the hardware. */
            /* Start HSMCI peripheral and card clock */
            atmel_hsmci_enable(hc);
            break;

        case PM_SLEEP:
            /* XXX: We might be able to save some more power by turning off the LDO output for the eMMC. */
            /* Stop HSMCI peripheral and card clock */
            atmel_hsmci_disable(hc);
            break;
    }

    return EOK;
}

/* Transfer read data in PIO mode */
static inline void atmel_hsmci_pio_in_xfer(sdio_hc_t *hc) {
    atmel_hc_hsmci_t *hsmci = (atmel_hc_hsmci_t*)hc->cs_hdl;

    while(in32(hsmci->base + ATMEL_HSMCI_SR) & ATMEL_HSMCI_SR_RXRDY) {
        uint8_t *addr;
        int      len;

        if(sdio_sg_nxt(hc, &addr, &len, 4)) {
            atmel_hsmci_interrupt_disable(hc, ATMEL_HSMCI_SR_RXRDY);
            break;
        }

        *((uint32_t*)addr) = in32(hsmci->base + ATMEL_HSMCI_RDR);
    }
}

/* Transfer write data in PIO mode */
static inline void atmel_hsmci_pio_out_xfer(sdio_hc_t *hc) {
    atmel_hc_hsmci_t *hsmci = (atmel_hc_hsmci_t*)hc->cs_hdl;

    while(in32(hsmci->base + ATMEL_HSMCI_SR) & ATMEL_HSMCI_SR_TXRDY) {
        uint8_t *addr;
        int      len;

        if(sdio_sg_nxt(hc, &addr, &len, 4)) {
            atmel_hsmci_interrupt_disable(hc, ATMEL_HSMCI_SR_TXRDY);
            break;
        }

        out32(hsmci->base + ATMEL_HSMCI_TDR, *((uint32_t*)addr));
    }
}

static int atmel_hsmci_cmd(sdio_hc_t *hc, sdio_cmd_t *cmd) {
    atmel_hc_hsmci_t *hsmci = (atmel_hc_hsmci_t*)hc->cs_hdl;

    /* These have to be set in this order to issue the command */
    uint32_t argr = 0;
    uint32_t cmdr = 0;
    uint32_t imr  = 0;

    /* Set command number */
    cmdr |= ATMEL_HSMCI_CMDR_CMDNB(cmd->opcode);

    /* Set response type */
    if(cmd->flags & SCF_RSP_PRESENT) {
        if(cmd->flags & SCF_RSP_136) {
            cmdr |= ATMEL_HSMCI_CMDR_RSPTYP_136;
            imr  |= ATMEL_HSMCI_SR_XFRDONE;
        } else if(cmd->flags & SCF_RSP_BUSY) {
            cmdr |= ATMEL_HSMCI_CMDR_RSPTYP_R1B;
            imr  |= ATMEL_HSMCI_SR_XFRDONE;
        } else {
            cmdr |= ATMEL_HSMCI_CMDR_RSPTYP_48;
            imr  |= ATMEL_HSMCI_SR_XFRDONE;
        }
    } else {
        cmdr |= ATMEL_HSMCI_CMDR_RSPTYP_NORESP;
        imr  |= ATMEL_HSMCI_SR_XFRDONE;
    }

    /* Set special command */
    cmdr |= ATMEL_HSMCI_CMDR_SPCMD_STD;

    /* Set open drain command */
    if(hc->bus_mode == BUS_MODE_OPEN_DRAIN) {
        cmdr |= ATMEL_HSMCI_CMDR_OPCMD;
    }

    /* Set max latency */
    cmdr |= ATMEL_HSMCI_CMDR_MAXLAT64;

    /* Set transfer command */
    if(cmd->flags & SCF_DATA_MSK) {
        if(cmd->flags & SCF_DATA_PHYS ) {
            /* Cannot handle physical addresses in PIO mode */
            return ENOTSUP;
        }
        /* Initialize PIO transfer */
        sdio_sg_start(hc, cmd->sgl, cmd->sgc);
        cmdr |= ATMEL_HSMCI_CMDR_TRCMD_START;
    } else if(cmd->opcode == MMC_STOP_TRANSMISSION) {
        cmdr |= ATMEL_HSMCI_CMDR_TRCMD_STOP;
    } else {
        cmdr |= ATMEL_HSMCI_CMDR_TRCMD_NO_DATA;
    }

    /* Set transfer direction */
    if(cmd->flags & SCF_DIR_IN) {
        cmdr |= ATMEL_HSMCI_CMDR_TDIR_READ;
        imr  |= ATMEL_HSMCI_SR_RXRDY;
    } else if(cmd->flags & SCF_DIR_OUT) {
        cmdr |= ATMEL_HSMCI_CMDR_TDIR_WRITE;
        imr  |= ATMEL_HSMCI_SR_TXRDY;
    }

    /* Set transfer type */
    if(cmd->blks > 1) {
        cmdr |= ATMEL_HSMCI_CMDR_TRTYP_MULTI;
    } else {
        cmdr |= ATMEL_HSMCI_CMDR_TRTYP_SINGLE;
    }

    /* Configure block register */
    out32(hsmci->base + ATMEL_HSMCI_BLKR,
        ATMEL_HSMCI_BLKR_BLKLEN(cmd->blksz) | ATMEL_HSMCI_BLKR_BCNT(cmd->blks));

    /* Set argument */
    argr = cmd->arg;

    /* Send command */
    out32(hsmci->base + ATMEL_HSMCI_ARGR, argr);
    out32(hsmci->base + ATMEL_HSMCI_CMDR, cmdr);
    atmel_hsmci_interrupt_enable (hc,  imr);
    atmel_hsmci_interrupt_disable(hc, ~imr);

    return EOK;
}

static int atmel_hsmci_abort(sdio_hc_t *hc, sdio_cmd_t *cmd) {
    atmel_hc_hsmci_t *hsmci = (atmel_hc_hsmci_t*)hc->cs_hdl;

    /* XXX: There is no good way to abort a command. */

    /* Disable interrupts */
    atmel_hsmci_interrupt_disable(hc, 0xFFFFFFFF);

    /* Wait for transfer done status. */
    while((in32(hsmci->base + ATMEL_HSMCI_SR) & ATMEL_HSMCI_SR_XFRDONE) == 0) {
        delay(1);
    }

    return EOK;
}

static int atmel_hsmci_event(sdio_hc_t *hc, sdio_event_t *ev) {
    atmel_hc_hsmci_t *hsmci = (atmel_hc_hsmci_t*)hc->cs_hdl;
    sdio_cmd_t       *cmd   = hc->wspc.cmd;

    /* In case we are called after dinit */
    if(hsmci == NULL) return EOK;

    uint32_t     sr = in32(hsmci->base + ATMEL_HSMCI_SR);
    uint32_t    imr = in32(hsmci->base + ATMEL_HSMCI_IMR);
    uint32_t reason = sr & imr;

    if(reason & ATMEL_HSMCI_SR_RXRDY) {
        atmel_hsmci_pio_in_xfer(hc);
    } else if(reason & ATMEL_HSMCI_SR_TXRDY) {
        atmel_hsmci_pio_out_xfer(hc);
    } else if(reason & ATMEL_HSMCI_SR_XFRDONE) {
        /* Transfer is done.
         * At this point, the command is ready, the data was transmitted, and the card is not busy.
         */

        /* Assume everything went well */
        uint32_t cs = CS_CMD_CMP;

        /* Check for errors, and get response */
        if((sr & ATMEL_HSMCI_SR_RTOE) && (cmd->flags & SCF_RSP_PRESENT)) {
            /* Response Time-out Error */
            cs = CS_CMD_TO_ERR;
        } else if((sr & ATMEL_HSMCI_SR_RCRCE) && (cmd->flags & SCF_RSP_CRC)) {
            /* Response CRC Error */
            cs = CS_CMD_CRC_ERR;
        } else if(sr & ATMEL_HSMCI_SR_RINDE) {
            /* Response Index Error */
            cs = CS_CMD_IDX_ERR;
        } else if(sr & ATMEL_HSMCI_SR_RENDE) {
            /* Response End Bit Error */
            cs = CS_CMD_END_ERR;
        } else if(sr & ATMEL_HSMCI_SR_DCRCE) {
            /* Data CRC Error */
            cs = CS_DATA_CRC_ERR;
        } else if(sr & ATMEL_HSMCI_SR_DTOE) {
            /* Data Time-out Error */
            cs = CS_DATA_TO_ERR;
        } else if(sr & ATMEL_HSMCI_SR_CSTOE) {
            /* Completion Signal Time-out Error */
            cs = CS_CMD_CMP_ERR;
        } else if(sr & ATMEL_HSMCI_SR_OERRORS) {
            /* XXX: Other errors that don't have SDMMC equivalents:
             *   - Response Direction Error
             *   - DMA Block Overrun Error
             *   - ACKRCVE: Boot Operation Acknowledge Error
             */
            cs = CS_CMD_END_ERR;
        } else if(cmd->flags & SCF_RSP_136) {
            cmd->rsp[0] = in32(hsmci->base + ATMEL_HSMCI_RSPR);
            cmd->rsp[1] = in32(hsmci->base + ATMEL_HSMCI_RSPR);
            cmd->rsp[2] = in32(hsmci->base + ATMEL_HSMCI_RSPR);
            cmd->rsp[3] = in32(hsmci->base + ATMEL_HSMCI_RSPR);
        } else if(cmd->flags & SCF_RSP_PRESENT) {
            cmd->rsp[0] = in32(hsmci->base + ATMEL_HSMCI_RSPR);
        }

        sdio_cmd_cmplt(hc, cmd, cs);
        atmel_hsmci_interrupt_disable(hc, imr);
    }

    InterruptUnmask(21, hc->hc_iid);
    return EOK;
}

static int atmel_hsmci_cd(sdio_hc_t *hc) {
    /* XXX: No real card detect for now.
     * The driver will be mainly used with eMMC, so card detect is not necessary.
     */
    return SDIO_TRUE;
}

static int atmel_hsmci_pwr(sdio_hc_t *hc, int vdd) {
    /* XXX: We don't actually set voltages.
     * This would have to be board specific and the eMMC is always at 1.8V anyway.
     */
    hc->vdd = vdd;
    return EOK;
}

static int atmel_hsmci_clk(sdio_hc_t *hc, int clk) {
    sdio_hc_cfg_t    *cfg   = &hc->cfg;
    atmel_hc_hsmci_t *hsmci = (atmel_hc_hsmci_t*)hc->cs_hdl;

    /* High Speed MultiMedia Card Interface clock (MCCK or HSMCI_CK) is Master Clock (MCK) divided by ({CLKDIV,CLKODD}+2). */

    clk = min(clk, cfg->clk/2);
    clk = max(clk, cfg->clk/(511+2));

    /* Calculate divider */
    unsigned divider = (cfg->clk)/(clk);
    while((cfg->clk/divider) > clk) {
        /* Increase divider to avoid overclocking the card. */
        divider++;
    }

    /* Calculate CLKDIV and CLKODD values. */
    unsigned clkdiv = (divider > 2) ? ((divider-2)/2) : 0;
    unsigned clkodd = (divider > 2) ? ((divider-2)%2) : 0;

    /* Configure MR */
    uint32_t mr = in32(hsmci->base + ATMEL_HSMCI_MR);
    mr &= ~(ATMEL_HSMCI_MR_CLKODD | ATMEL_HSMCI_MR_CLKDIV_MASK);
    mr |= ATMEL_HSMCI_MR_CLKDIV(clkdiv);
    mr |= clkodd ? ATMEL_HSMCI_MR_CLKODD : 0;
    out32(hsmci->base + ATMEL_HSMCI_MR, mr);

    /* Calculate real clock */
    hc->clk = cfg->clk/divider;

    return EOK;
}

static int atmel_hsmci_mode(sdio_hc_t *hc, int bus_mode) {
    hc->bus_mode = bus_mode;
    return EOK;
}

static int atmel_hsmci_bus_width(sdio_hc_t *hc, int width) {
    atmel_hc_hsmci_t *hsmci = (atmel_hc_hsmci_t*)hc->cs_hdl;

    uint32_t sdcr;
    switch(width) {
        default:
        case BUS_WIDTH_1:
            sdcr = ATMEL_HSMCI_SDCR_SDCBUS_BW1;
            break;
        case BUS_WIDTH_4:
            sdcr = ATMEL_HSMCI_SDCR_SDCBUS_BW4;
            break;
        case BUS_WIDTH_8:
            sdcr = ATMEL_HSMCI_SDCR_SDCBUS_BW8;
            break;
    }

    sdcr |= ATMEL_HSMCI_SDCR_SDCSEL_SLOTA;

    out32(hsmci->base + ATMEL_HSMCI_SDCR, sdcr);

    hc->bus_width = width;

    return EOK;
}

static int atmel_hsmci_timing(sdio_hc_t *hc, int timing) {
    atmel_hc_hsmci_t *hsmci = (atmel_hc_hsmci_t*)hc->cs_hdl;

    switch(timing) {
        default:
        case TIMING_HS:
            out32(hsmci->base + ATMEL_HSMCI_CFG, ATMEL_HSMCI_CFG_HSMODE);
            hc->timing = TIMING_HS;
            break;

        case TIMING_LS:
            out32(hsmci->base + ATMEL_HSMCI_CFG, 0);
            hc->timing = TIMING_LS;
            break;
    }

    return EINVAL;
}

static int atmel_hsmci_signal_voltage(sdio_hc_t *hc, int signal_voltage) {
    /* XXX: We don't actually set voltages.
     * This would have to be board specific and the eMMC is always at 1.8V anyway.
     */
    hc->signal_voltage = signal_voltage;
    return EOK;
}

int atmel_hsmci_dinit(sdio_hc_t *hc) {
    atmel_hc_hsmci_t *hsmci = (atmel_hc_hsmci_t*)hc->cs_hdl;

    /* Disable peripheral */
    atmel_hsmci_disable(hc);

    /* Detach Interrupt */
    (void)InterruptDetach(hc->hc_iid);

    /* Unmap peripheral */
    (void)munmap_device_io(hsmci->base, ATMEL_HSMCI_SIZE);

    /* Free allocated memory */
    free(hsmci);
    hc->cs_hdl = NULL;

    return EOK;
}

int atmel_hsmci_init(sdio_hc_t *hc) {
    sdio_hc_cfg_t *cfg = &hc->cfg;

    /* Set caps */
    hc->ocr     = OCR_VDD_32_33 | OCR_VDD_30_31 | OCR_VDD_29_30 | OCR_VDD_17_195;
    hc->caps   |= HC_CAP_BW4     | HC_CAP_BW8;
    hc->caps   |= HC_CAP_PIO;
    hc->caps   |= HC_CAP_HS;
    hc->caps   |= HC_CAP_SV_3_3V | HC_CAP_SV_1_8V;
    hc->caps   |= HC_CAP_SLEEP;
    hc->clk_max = 50000000;

    /* Find base address IRQ and clock */
    paddr_t   pbase = cfg->base_addr[0];
    unsigned  irq   = cfg->irq[0];

    /* Allocate data structure for chipset data */
    atmel_hc_hsmci_t *hsmci;
    hsmci = calloc(1, sizeof(*hsmci));
    if(hsmci == NULL) {
        return ENOMEM;
    }
    hc->cs_hdl = hsmci;

    /* Map HSMCI registers */
    hsmci->base = mmap_device_io(ATMEL_HSMCI_SIZE, (uint64_t)pbase);
    if(hsmci->base == MAP_DEVICE_FAILED) {
        errno_t saved_errno = errno;
        free(hsmci);
        return saved_errno;
    }

    /* Software reset */
    atmel_hsmci_swreset(hc);

    /* Attach interrupt */
    struct sigevent event;
    SIGEV_PULSE_INIT(&event, hc->hc_coid, SDIO_PRIORITY, HC_EV_INTR, NULL);
    if((hc->hc_iid = InterruptAttachEvent(irq, &event, _NTO_INTR_FLAGS_TRK_MSK)) == -1) {
        errno_t saved_errno = errno;
        (void)munmap_device_io(hsmci->base, ATMEL_HSMCI_SIZE);
        free(hsmci);
        return saved_errno;
    }

    /* Enable */
    atmel_hsmci_enable(hc);

    /* Set entry points */
    hc->entry = atmel_hsmci_hc_entry;

    /* DONE */
    return EOK;
}

#endif


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/6.6.0/trunk/hardware/devb/sdmmc/sdiodi/hc/atmel-hsmci.c $ $Rev: 789718 $")
#endif
