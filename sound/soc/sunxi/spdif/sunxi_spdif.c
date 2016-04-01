/*
 * sound\soc\sunxi\spdif\sunxi_spdif.c
 * (C) Copyright 2007-2011
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * chenpailin <chenpailin@allwinnertech.com>
 *
 * some simple description for this code
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/jiffies.h>
#include <linux/io.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include <mach/clock.h>
#include <plat/sys_config.h>

#include <mach/hardware.h>
#include <asm/dma.h>
#include <plat/dma_compat.h>

#include "sunxi_spdma.h"
#include "sunxi_spdif.h"

static int regsave[9];

static struct sunxi_dma_params sunxi_spdif_stereo_out = {
	.client.name	=	"SPDIF out",
#if defined CONFIG_ARCH_SUN4I || defined CONFIG_ARCH_SUN5I
	.channel	=	DMACH_NSPDIF,
#endif
	.dma_addr 	=	SUNXI_SPDIFBASE + SUNXI_SPDIF_TXFIFO,
};

static struct sunxi_dma_params sunxi_spdif_stereo_in = {
	.client.name	=	"SPDIF in",
#if defined CONFIG_ARCH_SUN4I || defined CONFIG_ARCH_SUN5I
	.channel	=	DMACH_NSPDIF, //???
#endif
	.dma_addr 	=	SUNXI_SPDIFBASE + SUNXI_SPDIF_RXFIFO,
};

struct sunxi_spdif_info sunxi_spdif;
static u32 spdif_handle = 0;
static struct clk *spdif_apbclk, *spdif_pll2clk, *spdif_pllx8, *spdif_moduleclk;

void sunxi_snd_txctrl(struct snd_pcm_substream *substream, int on)
{
	u32 reg_val;

	reg_val = readl(sunxi_spdif.regs + SUNXI_SPDIF_TXCFG);
	if (substream->runtime->channels == 1) {
		reg_val |= SUNXI_SPDIF_TXCFG_SINGLEMOD;
	}
	else {
		reg_val &= ~SUNXI_SPDIF_TXCFG_SINGLEMOD;
	}
	reg_val |= SUNXI_SPDIF_TXCFG_ASS;	//Sending the last audio (may be 0?)
	reg_val |= SUNXI_SPDIF_TXCFG_CHSTMODE;	//Channel status A&B generated form TX_CHSTA
	writel(reg_val, sunxi_spdif.regs + SUNXI_SPDIF_TXCFG);

	/*flush TX FIFO and set FIFO empty trigger level*/
	reg_val = readl(sunxi_spdif.regs + SUNXI_SPDIF_FCTL);
	reg_val |= SUNXI_SPDIF_FCTL_TXTL(0x10);	//TX FIFO empty Trigger Level
	reg_val |= SUNXI_SPDIF_FCTL_FTX;	//flush TX FIFO
	writel(reg_val, sunxi_spdif.regs + SUNXI_SPDIF_FCTL);

	/*clear interrupt status*/
	reg_val = readl(sunxi_spdif.regs + SUNXI_SPDIF_ISTA);
	reg_val |= SUNXI_SPDIF_ISTA_TXCLR;
	writel(reg_val, sunxi_spdif.regs + SUNXI_SPDIF_ISTA);

	/*clear TX counter*/
	writel(0, sunxi_spdif.regs + SUNXI_SPDIF_TXCNT);

	if (on) {
		//SPDIF TX ENABLE
		reg_val = readl(sunxi_spdif.regs + SUNXI_SPDIF_TXCFG);
		reg_val |= SUNXI_SPDIF_TXCFG_TXEN;
		writel(reg_val, sunxi_spdif.regs + SUNXI_SPDIF_TXCFG);

		//DRQ ENABLE
		reg_val = readl(sunxi_spdif.regs + SUNXI_SPDIF_INT);
		reg_val |= SUNXI_SPDIF_INT_TXDRQEN;
		writel(reg_val, sunxi_spdif.regs + SUNXI_SPDIF_INT);
	} else {
		//SPDIF TX DISABLE
		reg_val = readl(sunxi_spdif.regs + SUNXI_SPDIF_TXCFG);
		reg_val &= ~SUNXI_SPDIF_TXCFG_TXEN;
		writel(reg_val, sunxi_spdif.regs + SUNXI_SPDIF_TXCFG);

		//DRQ DISABLE
		reg_val = readl(sunxi_spdif.regs + SUNXI_SPDIF_INT);
		reg_val &= ~SUNXI_SPDIF_INT_TXDRQEN;
		writel(reg_val, sunxi_spdif.regs + SUNXI_SPDIF_INT);
	}
}

void sunxi_snd_rxctrl(struct snd_pcm_substream *substream, int on)
{
	u32 reg_val;

	/*flush RX FIFO and set FIFO empty trigger level*/
	reg_val = readl(sunxi_spdif.regs + SUNXI_SPDIF_FCTL);
	reg_val |= SUNXI_SPDIF_FCTL_RXTL(0x0F);	//RX FIFO Trigger Level
	reg_val |= SUNXI_SPDIF_FCTL_FRX;	//flush RX FIFO	
	writel(reg_val, sunxi_spdif.regs + SUNXI_SPDIF_FCTL);
	
	/*clear interrupt status*/
	reg_val = readl(sunxi_spdif.regs + SUNXI_SPDIF_ISTA);
	reg_val |= SUNXI_SPDIF_ISTA_RXCLR;
	writel(reg_val, sunxi_spdif.regs + SUNXI_SPDIF_ISTA);
	
	/*clear RX counter*/
	writel(0, sunxi_spdif.regs + SUNXI_SPDIF_RXCNT);

	if (on) {
		/*SPDIF RX ENABLE*/
		reg_val = readl(sunxi_spdif.regs + SUNXI_SPDIF_RXCFG);
		reg_val |= SUNXI_SPDIF_RXCFG_RXEN;	
		writel(reg_val, sunxi_spdif.regs + SUNXI_SPDIF_RXCFG);
		
		/*DRQ ENABLE*/
		reg_val = readl(sunxi_spdif.regs + SUNXI_SPDIF_INT);
		reg_val |= SUNXI_SPDIF_INT_RXDRQEN;	
		writel(reg_val, sunxi_spdif.regs + SUNXI_SPDIF_INT);
	} else {
		/*SPDIF TX DISABLE*/
		reg_val = readl(sunxi_spdif.regs + SUNXI_SPDIF_RXCFG);
		reg_val &= ~SUNXI_SPDIF_RXCFG_RXEN;	
		writel(reg_val, sunxi_spdif.regs + SUNXI_SPDIF_RXCFG);
		
		/*DRQ DISABLE*/
		reg_val = readl(sunxi_spdif.regs + SUNXI_SPDIF_INT);
		reg_val &= ~SUNXI_SPDIF_INT_RXDRQEN;	
		writel(reg_val, sunxi_spdif.regs + SUNXI_SPDIF_INT);
	}
}

static inline int sunxi_snd_is_clkmaster(void)
{
	return 0;
}

static int sunxi_spdif_set_fmt(struct snd_soc_dai *cpu_dai, unsigned int fmt)
{
	u32 reg_val;

	if (!fmt) {//PCM
		reg_val = readl(sunxi_spdif.regs + SUNXI_SPDIF_TXCFG);
		reg_val &= ~SUNXI_SPDIF_TXCFG_NONAUDIO;
		writel(reg_val, sunxi_spdif.regs + SUNXI_SPDIF_TXCFG);

		reg_val = readl(sunxi_spdif.regs + SUNXI_SPDIF_TXCHSTA0);
		reg_val |= (SUNXI_SPDIF_TXCHSTA0_CHNUM(2));
		reg_val &= ~SUNXI_SPDIF_TXCHSTA0_AUDIO;
		writel(reg_val, sunxi_spdif.regs + SUNXI_SPDIF_TXCHSTA0);

		reg_val = readl(sunxi_spdif.regs + SUNXI_SPDIF_RXCHSTA0);
		reg_val |= (SUNXI_SPDIF_RXCHSTA0_CHNUM(2));
		reg_val &= ~SUNXI_SPDIF_RXCHSTA0_AUDIO;
		writel(reg_val, sunxi_spdif.regs + SUNXI_SPDIF_RXCHSTA0);
	} else {  //non PCM
		reg_val = readl(sunxi_spdif.regs + SUNXI_SPDIF_TXCFG);
		reg_val |= SUNXI_SPDIF_TXCFG_NONAUDIO;
		writel(reg_val, sunxi_spdif.regs + SUNXI_SPDIF_TXCFG);

		reg_val = readl(sunxi_spdif.regs + SUNXI_SPDIF_TXCHSTA0);
		reg_val |= (SUNXI_SPDIF_TXCHSTA0_CHNUM(2));
		reg_val |= SUNXI_SPDIF_TXCHSTA0_AUDIO;
		writel(reg_val, sunxi_spdif.regs + SUNXI_SPDIF_TXCHSTA0);

		reg_val = readl(sunxi_spdif.regs + SUNXI_SPDIF_RXCHSTA0);
		reg_val |= (SUNXI_SPDIF_RXCHSTA0_CHNUM(2));
		reg_val |= SUNXI_SPDIF_RXCHSTA0_AUDIO;
		writel(reg_val, sunxi_spdif.regs + SUNXI_SPDIF_RXCHSTA0);
	}

	return 0;
}

static int sunxi_spdif_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct sunxi_dma_params *dma_data;
	u32 reg_val, reg_val1;
	int format;
	switch (params_format(params))
	{
	case SNDRV_PCM_FORMAT_S16_LE:
		format = 16;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		format = 20;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		format = 24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		format = 24;
		break;
	default:
		return -EINVAL;
	}

	/* playback or capture */
	if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		dma_data = &sunxi_spdif_stereo_out;

		reg_val1 = readl(sunxi_spdif.regs + SUNXI_SPDIF_TXCHSTA1);
		reg_val = readl(sunxi_spdif.regs + SUNXI_SPDIF_TXCFG);
		reg_val &= ~SUNXI_SPDIF_TXCFG_FMTRVD;
		reg_val1 &= ~SUNXI_SPDIF_TXCHSTA1_MAXWORDLEN;
		reg_val1 &= ~(SUNXI_SPDIF_TXCHSTA1_SAMWORDLEN(7));
		if(format == 16) {
			reg_val |= SUNXI_SPDIF_TXCFG_FMT16BIT;
			reg_val1 |= (SUNXI_SPDIF_TXCHSTA1_SAMWORDLEN(1));
		}
		else if(format == 20) {
			reg_val |= SUNXI_SPDIF_TXCFG_FMT20BIT;
			reg_val1 |= SUNXI_SPDIF_TXCHSTA1_MAXWORDLEN;
			reg_val1 |= (SUNXI_SPDIF_TXCHSTA1_SAMWORDLEN(1));
		}
		else {
			reg_val |= SUNXI_SPDIF_TXCFG_FMT24BIT;
			reg_val1 |= SUNXI_SPDIF_TXCHSTA1_MAXWORDLEN;
			reg_val1 |= (SUNXI_SPDIF_TXCHSTA1_SAMWORDLEN(5));
		}
		writel(reg_val, sunxi_spdif.regs + SUNXI_SPDIF_TXCFG);
		writel(reg_val1, sunxi_spdif.regs + SUNXI_SPDIF_TXCHSTA1);

		/* Set TX FIFO Input Mode */
		reg_val = readl(sunxi_spdif.regs + SUNXI_SPDIF_FCTL);
		reg_val |= SUNXI_SPDIF_FCTL_TXIM1;	//1. Valid data at the LSB of OWA_TXFIFO register
		//reg_val &= ~SUNXI_SPDIF_FCTL_TXIM1;	//0. Valid data at the MSB of OWA_TXFIFO register
		writel(reg_val, sunxi_spdif.regs + SUNXI_SPDIF_FCTL);
	}
	else {
		dma_data = &sunxi_spdif_stereo_in;
		/* Set TX FIFO Input Mode */
		reg_val1 = readl(sunxi_spdif.regs + SUNXI_SPDIF_RXCHSTA1);
		reg_val = readl(sunxi_spdif.regs + SUNXI_SPDIF_FCTL);
		reg_val &= ~SUNXI_SPDIF_FCTL_RXOM3;
		reg_val1 &= ~SUNXI_SPDIF_RXCHSTA1_MAXWORDLEN;
		reg_val1 &= ~(SUNXI_SPDIF_RXCHSTA1_SAMWORDLEN(7));

		if(format == 16) {
			reg_val |= SUNXI_SPDIF_FCTL_RXOM3;
			reg_val1 |= (SUNXI_SPDIF_RXCHSTA1_SAMWORDLEN(1));
		}
		else if(format == 20) {
			reg_val1 |= SUNXI_SPDIF_RXCHSTA1_MAXWORDLEN;
			reg_val1 |= (SUNXI_SPDIF_RXCHSTA1_SAMWORDLEN(1));
		}
		else {
			reg_val1 |= SUNXI_SPDIF_RXCHSTA1_MAXWORDLEN;
			reg_val1 |= (SUNXI_SPDIF_RXCHSTA1_SAMWORDLEN(5));
		}

		writel(reg_val, sunxi_spdif.regs + SUNXI_SPDIF_FCTL);
		writel(reg_val1, sunxi_spdif.regs + SUNXI_SPDIF_RXCHSTA1);
	}
	snd_soc_dai_set_dma_data(rtd->cpu_dai, substream, dma_data);
	return 0;
}

static int sunxi_spdif_trigger(struct snd_pcm_substream *substream,
                              int cmd, struct snd_soc_dai *dai)
{
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct sunxi_dma_params *dma_data =
					snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);

	switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
		case SNDRV_PCM_TRIGGER_RESUME:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
				sunxi_snd_rxctrl(substream, 1);
			} else {
				sunxi_snd_txctrl(substream, 1);
			}
			sunxi_dma_started(dma_data);
			break;
		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
			if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
				sunxi_snd_rxctrl(substream, 0);
			} else {
				sunxi_snd_txctrl(substream, 0);
			}
			break;
		default:
			ret = -EINVAL;
			break;
	}

		return ret;
}

//freq:   1: 22.5792MHz   0: 24.576MHz
static int sunxi_spdif_set_sysclk(struct snd_soc_dai *cpu_dai, int clk_id,
                                 unsigned int freq, int dir)
{
	if (!freq) {
		clk_set_rate(spdif_pll2clk, 24576000);
	} else {
		clk_set_rate(spdif_pll2clk, 22579200);
	}

	return 0;
}

static int sunxi_spdif_set_clkdiv(struct snd_soc_dai *cpu_dai, int div_id, int div)
{
	u32 reg_val_txchsta0 = 0;
	u32 reg_val_txchsta1 = 0;
	u32 reg_val_rxchsta0 = 0;
	u32 reg_val_rxchsta1 = 0;
	u32 reg_val_txcfg = 0;

	switch(div_id) {
		case SUNXI_DIV_MCLK:
		{
			reg_val_txchsta0 = readl(sunxi_spdif.regs + SUNXI_SPDIF_TXCHSTA0);
			reg_val_txchsta0 &= ~(SUNXI_SPDIF_TXCHSTA0_SAMFREQ(0xf));

			reg_val_txchsta1 = readl(sunxi_spdif.regs + SUNXI_SPDIF_TXCHSTA1);
			reg_val_txchsta1 &= ~(SUNXI_SPDIF_TXCHSTA1_ORISAMFREQ(0xf));

			reg_val_rxchsta0 = readl(sunxi_spdif.regs + SUNXI_SPDIF_RXCHSTA0);
			reg_val_rxchsta0 &= ~(SUNXI_SPDIF_RXCHSTA0_SAMFREQ(0xf));

			reg_val_rxchsta1 = readl(sunxi_spdif.regs + SUNXI_SPDIF_RXCHSTA1);
			reg_val_rxchsta1 &= ~(SUNXI_SPDIF_RXCHSTA1_ORISAMFREQ(0xf));

			reg_val_txcfg = readl(sunxi_spdif.regs + SUNXI_SPDIF_TXCFG);
			reg_val_txcfg &= ~SUNXI_SPDIF_TXCFG_TXRATIO(0x1F);
			reg_val_txcfg |= SUNXI_SPDIF_TXCFG_TXRATIO(div-1);
			writel(reg_val_txcfg, sunxi_spdif.regs + SUNXI_SPDIF_TXCFG);

			if(clk_get_rate(spdif_pll2clk) == 24576000)
			{
				switch(div)
				{
					//24KHZ
					case 8:
						reg_val_txchsta0 |= (SUNXI_SPDIF_TXCHSTA0_SAMFREQ(0x6));
						reg_val_txchsta1 |= (SUNXI_SPDIF_TXCHSTA1_ORISAMFREQ(0x9));
						reg_val_rxchsta0 |= (SUNXI_SPDIF_RXCHSTA0_SAMFREQ(0x6));
						reg_val_rxchsta1 |= (SUNXI_SPDIF_RXCHSTA1_ORISAMFREQ(0x9));
						break;

					//32KHZ
					case 6:
						reg_val_txchsta0 |= (SUNXI_SPDIF_TXCHSTA0_SAMFREQ(0x3));
						reg_val_txchsta1 |= (SUNXI_SPDIF_TXCHSTA1_ORISAMFREQ(0xC));
						reg_val_rxchsta0 |= (SUNXI_SPDIF_RXCHSTA0_SAMFREQ(0x3));
						reg_val_rxchsta1 |= (SUNXI_SPDIF_RXCHSTA1_ORISAMFREQ(0xC));
						break;

					//48KHZ
					case 4:
						reg_val_txchsta0 |= (SUNXI_SPDIF_TXCHSTA0_SAMFREQ(0x2));
						reg_val_txchsta1 |= (SUNXI_SPDIF_TXCHSTA1_ORISAMFREQ(0xD));
						reg_val_rxchsta0 |= (SUNXI_SPDIF_RXCHSTA0_SAMFREQ(0x2));
						reg_val_rxchsta1 |= (SUNXI_SPDIF_RXCHSTA1_ORISAMFREQ(0xD));
						break;

					//96KHZ
					case 2:
						reg_val_txchsta0 |= (SUNXI_SPDIF_TXCHSTA0_SAMFREQ(0xA));
						reg_val_txchsta1 |= (SUNXI_SPDIF_TXCHSTA1_ORISAMFREQ(0x5));
						reg_val_rxchsta0 |= (SUNXI_SPDIF_RXCHSTA0_SAMFREQ(0xA));
						reg_val_rxchsta1 |= (SUNXI_SPDIF_RXCHSTA1_ORISAMFREQ(0x5));
						break;

					//192KHZ
					case 1:
						reg_val_txchsta0 |= (SUNXI_SPDIF_TXCHSTA0_SAMFREQ(0xE));
						reg_val_txchsta1 |= (SUNXI_SPDIF_TXCHSTA1_ORISAMFREQ(0x1));
						reg_val_rxchsta0 |= (SUNXI_SPDIF_RXCHSTA0_SAMFREQ(0xE));
						reg_val_rxchsta1 |= (SUNXI_SPDIF_RXCHSTA1_ORISAMFREQ(0x1));
						break;

					default:
						reg_val_txchsta0 |= (SUNXI_SPDIF_TXCHSTA0_SAMFREQ(1));
						reg_val_txchsta1 |= (SUNXI_SPDIF_TXCHSTA1_ORISAMFREQ(0));
						reg_val_rxchsta0 |= (SUNXI_SPDIF_RXCHSTA0_SAMFREQ(1));
						reg_val_rxchsta1 |= (SUNXI_SPDIF_RXCHSTA1_ORISAMFREQ(0));
						break;
				}
			}else{  //22.5792MHz
				switch(div)
				{
					//22.05khz
					case 8:
						reg_val_txchsta0 |= (SUNXI_SPDIF_TXCHSTA0_SAMFREQ(0x4));
						reg_val_txchsta1 |= (SUNXI_SPDIF_TXCHSTA1_ORISAMFREQ(0xb));
						reg_val_rxchsta0 |= (SUNXI_SPDIF_RXCHSTA0_SAMFREQ(0x4));
						reg_val_rxchsta1 |= (SUNXI_SPDIF_RXCHSTA1_ORISAMFREQ(0xb));
						break;

					//44.1KHZ
					case 4:
						reg_val_txchsta0 |= (SUNXI_SPDIF_TXCHSTA0_SAMFREQ(0x0));
						reg_val_txchsta1 |= (SUNXI_SPDIF_TXCHSTA1_ORISAMFREQ(0xF));
						reg_val_rxchsta0 |= (SUNXI_SPDIF_RXCHSTA0_SAMFREQ(0x0));
						reg_val_rxchsta1 |= (SUNXI_SPDIF_RXCHSTA1_ORISAMFREQ(0xF));
						break;

					//88.2khz
					case 2:
						reg_val_txchsta0 |= (SUNXI_SPDIF_TXCHSTA0_SAMFREQ(0x8));
						reg_val_txchsta1 |= (SUNXI_SPDIF_TXCHSTA1_ORISAMFREQ(0x7));
						reg_val_rxchsta0 |= (SUNXI_SPDIF_RXCHSTA0_SAMFREQ(0x8));
						reg_val_rxchsta1 |= (SUNXI_SPDIF_RXCHSTA1_ORISAMFREQ(0x7));
						break;

					//176.4KHZ
					case 1:
						reg_val_txchsta0 |= (SUNXI_SPDIF_TXCHSTA0_SAMFREQ(0xC));
						reg_val_txchsta1 |= (SUNXI_SPDIF_TXCHSTA1_ORISAMFREQ(0x3));
						reg_val_rxchsta0 |= (SUNXI_SPDIF_RXCHSTA0_SAMFREQ(0xC));
						reg_val_rxchsta1 |= (SUNXI_SPDIF_RXCHSTA1_ORISAMFREQ(0x3));
						break;

					default:
						reg_val_txchsta0 |= (SUNXI_SPDIF_TXCHSTA0_SAMFREQ(1));
						reg_val_txchsta1 |= (SUNXI_SPDIF_TXCHSTA1_ORISAMFREQ(0));
						reg_val_rxchsta0 |= (SUNXI_SPDIF_RXCHSTA0_SAMFREQ(1));
						reg_val_rxchsta1 |= (SUNXI_SPDIF_RXCHSTA1_ORISAMFREQ(0));
						
						break;
				}
			}
			writel(reg_val_txchsta0, sunxi_spdif.regs + SUNXI_SPDIF_TXCHSTA0);
		  	writel(reg_val_txchsta1, sunxi_spdif.regs + SUNXI_SPDIF_TXCHSTA1);

			writel(reg_val_rxchsta0, sunxi_spdif.regs + SUNXI_SPDIF_RXCHSTA0);
		  	writel(reg_val_rxchsta1, sunxi_spdif.regs + SUNXI_SPDIF_RXCHSTA1);
		}
		break;
		case SUNXI_DIV_BCLK:
		break;

		default:
			return -EINVAL;
	}
	return 0;
}

u32 sunxi_spdif_get_clockrate(void)
{
	return 0;
}
EXPORT_SYMBOL_GPL(sunxi_spdif_get_clockrate);

static int sunxi_spdif_dai_probe(struct snd_soc_dai *dai)
{
	return 0;
}
static int sunxi_spdif_dai_remove(struct snd_soc_dai *dai)
{
	return 0;
}

static void spdifregsave(void)
{
	regsave[0] = readl(sunxi_spdif.regs + SUNXI_SPDIF_CTL);
	regsave[1] = readl(sunxi_spdif.regs + SUNXI_SPDIF_TXCFG);
	regsave[2] = readl(sunxi_spdif.regs + SUNXI_SPDIF_FCTL) | (0x3<<16);	//clear TX, RX fifo
	regsave[3] = readl(sunxi_spdif.regs + SUNXI_SPDIF_INT);
	regsave[4] = readl(sunxi_spdif.regs + SUNXI_SPDIF_TXCHSTA0);
	regsave[5] = readl(sunxi_spdif.regs + SUNXI_SPDIF_TXCHSTA1);
	regsave[6] = readl(sunxi_spdif.regs + SUNXI_SPDIF_RXCFG);
	regsave[7] = readl(sunxi_spdif.regs + SUNXI_SPDIF_RXCHSTA0);
	regsave[8] = readl(sunxi_spdif.regs + SUNXI_SPDIF_RXCHSTA1);	
}

static void spdifregrestore(void)
{
	writel(regsave[0], sunxi_spdif.regs + SUNXI_SPDIF_CTL);
	writel(regsave[1], sunxi_spdif.regs + SUNXI_SPDIF_TXCFG);
	writel(regsave[2], sunxi_spdif.regs + SUNXI_SPDIF_FCTL);
	writel(regsave[3], sunxi_spdif.regs + SUNXI_SPDIF_INT);
	writel(regsave[4], sunxi_spdif.regs + SUNXI_SPDIF_TXCHSTA0);
	writel(regsave[5], sunxi_spdif.regs + SUNXI_SPDIF_TXCHSTA1);
	writel(regsave[6], sunxi_spdif.regs + SUNXI_SPDIF_RXCFG);
	writel(regsave[7], sunxi_spdif.regs + SUNXI_SPDIF_RXCHSTA0);
	writel(regsave[8], sunxi_spdif.regs + SUNXI_SPDIF_RXCHSTA1);
}

//#ifdef CONFIG_PM
static int sunxi_spdif_suspend(struct snd_soc_dai *cpu_dai)
{
	u32 reg_val;
	printk("[SPDIF] Enter %s\n", __func__);

	//global disable
	reg_val = readl(sunxi_spdif.regs + SUNXI_SPDIF_CTL);
	reg_val &= ~SUNXI_SPDIF_CTL_GEN;
	writel(reg_val, sunxi_spdif.regs + SUNXI_SPDIF_CTL);

	spdifregsave();

	//disable the module clock
	clk_disable(spdif_moduleclk);

	//clear start?
	clk_disable(spdif_apbclk);
	//clear end?

	printk("[SPDIF]SPECIAL CLK 0x01c20068 = %#x, line= %d\n", *(volatile int*)0xF1C20068, __LINE__);
	printk("[SPDIF]SPECIAL CLK 0x01c200C0 = %#x, line= %d\n", *(volatile int*)0xF1C200C0, __LINE__);

	return 0;
}

static int sunxi_spdif_resume(struct snd_soc_dai *cpu_dai)
{
	u32 reg_val;
	printk("[SPDIF] Enter %s\n", __func__);

	//enable the module clock
	clk_enable(spdif_apbclk);

	//enable the module clock
	clk_enable(spdif_moduleclk);

	spdifregrestore();

	reg_val = readl(sunxi_spdif.regs + SUNXI_SPDIF_FCTL);
	reg_val &= ~SUNXI_SPDIF_FCTL_FIFOSRC;	//set TX FIFO source select as APB bus
	writel(reg_val, sunxi_spdif.regs + SUNXI_SPDIF_FCTL);

	reg_val = readl(sunxi_spdif.regs + SUNXI_SPDIF_CTL);
	//soft reset SPDIF
	reg_val |= SUNXI_SPDIF_CTL_RESET;
	//global enable
	reg_val |= SUNXI_SPDIF_CTL_GEN;
	writel(reg_val, sunxi_spdif.regs + SUNXI_SPDIF_CTL);

	printk("[SPDIF]PLL2 0x01c20008 = %#x\n", *(volatile int*)0xF1C20008);
	printk("[SPDIF]SPECIAL CLK 0x01c20068 = %#x, line= %d\n", *(volatile int*)0xF1C20068, __LINE__);
	printk("[SPDIF]SPECIAL CLK 0x01c200C0 = %#x, line = %d\n", *(volatile int*)0xF1C200C0, __LINE__);

	return 0;
}

#define SUNXI_SPDIF_RATES (SNDRV_PCM_RATE_8000_192000 | SNDRV_PCM_RATE_KNOT)
static struct snd_soc_dai_ops sunxi_spdif_dai_ops = {
	.trigger 	= sunxi_spdif_trigger,
	.hw_params 	= sunxi_spdif_hw_params,
	.set_fmt 	= sunxi_spdif_set_fmt,
	.set_clkdiv 	= sunxi_spdif_set_clkdiv,
	.set_sysclk 	= sunxi_spdif_set_sysclk,
};
static struct snd_soc_dai_driver sunxi_spdif_dai = {
	.probe 		= sunxi_spdif_dai_probe,
	.suspend 	= sunxi_spdif_suspend,
	.resume 	= sunxi_spdif_resume,
	.remove 	= sunxi_spdif_dai_remove,
	.playback = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = SUNXI_SPDIF_RATES,
	.formats = SNDRV_PCM_FMTBIT_S16_LE|SNDRV_PCM_FMTBIT_S20_3LE| SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE,},
	.capture = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = SUNXI_SPDIF_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE|SNDRV_PCM_FMTBIT_S20_3LE| SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE,},
	.ops = &sunxi_spdif_dai_ops,
	.symmetric_rates = 1,
};

static int __devinit sunxi_spdif_dev_probe(struct platform_device *pdev)
{
	int reg_val = 0;
	int ret = 0;

	sunxi_spdif.regs = ioremap(SUNXI_SPDIFBASE, 0x100);
	if(sunxi_spdif.regs == NULL)
		return -ENXIO;

	//spdif apbclk
	spdif_apbclk = clk_get(NULL, "apb_spdif");
	if(-1 == clk_enable(spdif_apbclk)){
		printk("spdif_apbclk failed! line = %d\n", __LINE__);
	}

	spdif_pllx8 = clk_get(NULL, "audio_pllx8");

	//spdif pll2clk
	spdif_pll2clk = clk_get(NULL, "audio_pll");

	//spdif module clk
	spdif_moduleclk = clk_get(NULL, "spdif");

	if(clk_set_parent(spdif_moduleclk, spdif_pll2clk)){
		printk("try to set parent of spdif_moduleclk to spdif_pll2ck failed! line = %d\n",__LINE__);
	}

	if(clk_set_rate(spdif_moduleclk, 24576000/8)){
		printk("set spdif_moduleclk clock freq to 24576000 failed! line = %d\n", __LINE__);
	}

	if(-1 == clk_enable(spdif_moduleclk)){
		printk("open spdif_moduleclk failed! line = %d\n", __LINE__);
	}

	reg_val = readl(sunxi_spdif.regs + SUNXI_SPDIF_FCTL);
	reg_val &= ~SUNXI_SPDIF_FCTL_FIFOSRC;	//set TX FIFO source select as APB bus
	writel(reg_val, sunxi_spdif.regs + SUNXI_SPDIF_FCTL);

	reg_val = readl(sunxi_spdif.regs + SUNXI_SPDIF_CTL);
	//soft reset SPDIF
	reg_val |= SUNXI_SPDIF_CTL_RESET;
	//global enable
	reg_val |= SUNXI_SPDIF_CTL_GEN;
	writel(reg_val, sunxi_spdif.regs + SUNXI_SPDIF_CTL);

	ret = snd_soc_register_dai(&pdev->dev, &sunxi_spdif_dai);

	iounmap(sunxi_spdif.ioregs);

	return 0;
}

static int __devexit sunxi_spdif_dev_remove(struct platform_device *pdev)
{
	int reg_val = 0;
	//global disable
	reg_val = readl(sunxi_spdif.regs + SUNXI_SPDIF_CTL);
	reg_val &= ~SUNXI_SPDIF_CTL_GEN;
	writel(reg_val, sunxi_spdif.regs + SUNXI_SPDIF_CTL);

	/* release the module clock */
	clk_disable(spdif_moduleclk);

	/* release pllx8clk */
	clk_put(spdif_pllx8);

	/* release pll2clk */
	clk_put(spdif_pll2clk);

	/* release apbclk */
	clk_put(spdif_apbclk);

	gpio_release(spdif_handle, 2);
	snd_soc_unregister_dai(&pdev->dev);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_device sunxi_spdif_device = {
	.name = "sunxi-spdif",
};

static struct platform_driver sunxi_spdif_driver = {
	.probe = sunxi_spdif_dev_probe,
	.remove = __devexit_p(sunxi_spdif_dev_remove),
	.driver = {
		.name = "sunxi-spdif",
		.owner = THIS_MODULE,
	},
};

static int __init sunxi_spdif_init(void)
{
	int ret, spdif_used = 0;

	ret = script_parser_fetch("spdif_para", "spdif_used", &spdif_used, 1);
	if (ret != 0 || !spdif_used)
		return -ENODEV;

	spdif_handle = gpio_request_ex("spdif_para", NULL);

	ret = platform_device_register(&sunxi_spdif_device);
	if (ret < 0)
		return ret;

	ret = platform_driver_register(&sunxi_spdif_driver);
	if (ret < 0) {
		platform_device_unregister(&sunxi_spdif_device);
		return ret;
	}
	return 0;
}
module_init(sunxi_spdif_init);

static void __exit sunxi_spdif_exit(void)
{
	platform_driver_unregister(&sunxi_spdif_driver);
	platform_device_unregister(&sunxi_spdif_device);
}
module_exit(sunxi_spdif_exit);

/* Module information */
MODULE_AUTHOR("ALLWINNER");
MODULE_DESCRIPTION("sunxi SPDIF SoC Interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:sunxi-spdif");
