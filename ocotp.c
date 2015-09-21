/*
 * Freescale On-Chip OTP driver
 *
 * Copyright (C) 2010-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 * Original Author: Huang Shijie <b32955@freescale.com>
 * Copyright (C) 2015 Athom B.V. All Rights Reserved.
 * Author: Jeroen Vollenbrock <jeroen@athom.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h> 
#include <sys/mman.h>
#include <fcntl.h>
#include <byteswap.h>
#include <unistd.h>
#include <string.h>

#define PROC_OCOTP_PATH                        "/proc/device-tree/soc/aips-bus@02100000/ocotp@021bc000/reg"
#define PROC_CCM_PATH                         "/proc/device-tree/soc/aips-bus@02000000/ccm@020c4000/reg"

#ifdef DEBUG
#define debug(...) fprintf(stderr, __VA_ARGS__)
#else
#define debug(...)
#endif

#define error(...) fprintf(stderr, __VA_ARGS__)

#define HW_CCM_CBCDR                     0x00000014
#define BP_CCM_CBCDR_IPG_PODF            8
#define BM_CCM_CBCDR_IPG_PODF            0x00000300
#define BP_CCM_CBCDR_AHB_PODF            10
#define BM_CCM_CBCDR_AHB_PODF            0x00001C00

#define HW_CCM_CBCMR                     0x00000018
#define BP_CCM_CBCMR_PRE_PERIPH_CLK_SEL  18
#define BM_CCM_CBCMR_PRE_PERIPH_CLK_SEL  0x000C0000

#define HW_CCM_CGR2                      0x00000070
#define BP_CCM_CGR2_OCOTP                12
#define BM_CCM_CGR2_OCOTP                0x00003000

#define HW_OCOTP_CTRL                    0x00000000
#define HW_OCOTP_CTRL_SET                0x00000004
#define BP_OCOTP_CTRL_WR_UNLOCK          16
#define BM_OCOTP_CTRL_WR_UNLOCK          0xFFFF0000
#define BM_OCOTP_CTRL_RELOAD_SHADOWS     0x00000400
#define BM_OCOTP_CTRL_ERROR              0x00000200
#define BM_OCOTP_CTRL_BUSY               0x00000100
#define BP_OCOTP_CTRL_ADDR               0
#define BM_OCOTP_CTRL_ADDR               0x0000007F

#define HW_OCOTP_CTRL_CLR                0x00000008

#define HW_OCOTP_TIMING                  0x00000010
#define BP_OCOTP_TIMING_STROBE_READ      16
#define BM_OCOTP_TIMING_STROBE_READ      0x003F0000
#define BP_OCOTP_TIMING_RELAX            12
#define BM_OCOTP_TIMING_RELAX            0x0000F000
#define BP_OCOTP_TIMING_STROBE_PROG      0
#define BM_OCOTP_TIMING_STROBE_PROG      0x00000FFF

#define HW_OCOTP_DATA                    0x00000020

#define HW_OCOTP_CUST_N(n)               (0x00000400 + (n) * 0x10)
#define BF(value, field)                 (((value) << BP_##field) & BM_##field)
#define FB(value, field)                 (((value) & BM_##field) >> BP_##field)

#define DEF_RELAX                        20    /* > 16.5ns */

#define BANK(a, b, c, d, e, f, g, h) { \
    "HW_OCOTP_"#a, "HW_OCOTP_"#b, "HW_OCOTP_"#c, "HW_OCOTP_"#d, \
    "HW_OCOTP_"#e, "HW_OCOTP_"#f, "HW_OCOTP_"#g, "HW_OCOTP_"#h, \
}

static const uint32_t ccm_pre_periph_clk_sources[] = {528000000, 396000000, 306580000, 198000000};

static const char *imx6q_otp_desc[16][8] = {
    BANK( LOCK,       CFG0,       CFG1,       CFG2,       CFG3,       CFG4,       CFG5,       CFG6       ),
    BANK( MEM0,       MEM1,       MEM2,       MEM3,       MEM4,       ANA0,       ANA1,       ANA2       ),
    BANK( OTPMK0,     OTPMK1,     OTPMK2,     OTPMK3,     OTPMK4,     OTPMK5,     OTPMK6,     OTPMK7     ),
    BANK( SRK0,       SRK1,       SRK2,       SRK3,       SRK4,       SRK5,       SRK6,       SRK7       ),
    BANK( RESP0,      HSJC_RESP1, MAC0,       MAC1,       HDCP_KSV0,  HDCP_KSV1,  GP1,        GP2        ),
    BANK( DTCP_KEY0,  DTCP_KEY1,  DTCP_KEY2,  DTCP_KEY3,  DTCP_KEY4,  MISC_CONF,  FIELD_RETURN, SRK_REVOKE ),
    BANK( HDCP_KEY0,  HDCP_KEY1,  HDCP_KEY2,  HDCP_KEY3,  HDCP_KEY4,  HDCP_KEY5,  HDCP_KEY6,  HDCP_KEY7  ),
    BANK( HDCP_KEY8,  HDCP_KEY9,  HDCP_KEY10, HDCP_KEY11, HDCP_KEY12, HDCP_KEY13, HDCP_KEY14, HDCP_KEY15 ),
    BANK( HDCP_KEY16, HDCP_KEY17, HDCP_KEY18, HDCP_KEY19, HDCP_KEY20, HDCP_KEY21, HDCP_KEY22, HDCP_KEY23 ),
    BANK( HDCP_KEY24, HDCP_KEY25, HDCP_KEY26, HDCP_KEY27, HDCP_KEY28, HDCP_KEY29, HDCP_KEY30, HDCP_KEY31 ),
    BANK( HDCP_KEY32, HDCP_KEY33, HDCP_KEY34, HDCP_KEY35, HDCP_KEY36, HDCP_KEY37, HDCP_KEY38, HDCP_KEY39 ),
    BANK( HDCP_KEY40, HDCP_KEY41, HDCP_KEY42, HDCP_KEY43, HDCP_KEY44, HDCP_KEY45, HDCP_KEY46, HDCP_KEY47 ),
    BANK( HDCP_KEY48, HDCP_KEY49, HDCP_KEY50, HDCP_KEY51, HDCP_KEY52, HDCP_KEY53, HDCP_KEY54, HDCP_KEY55 ),
    BANK( HDCP_KEY56, HDCP_KEY57, HDCP_KEY58, HDCP_KEY59, HDCP_KEY60, HDCP_KEY61, HDCP_KEY62, HDCP_KEY63 ),
    BANK( HDCP_KEY64, HDCP_KEY65, HDCP_KEY66, HDCP_KEY67, HDCP_KEY68, HDCP_KEY69, HDCP_KEY70, HDCP_KEY71 ),
    BANK( CRC0,       CRC1,       CRC2,       CRC3,       CRC4,       CRC5,       CRC6,       CRC7       ),
};

struct reg_data_t {
    uint32_t addr;
    uint32_t size;
};

static void *otp_base, *ccm_base;
static struct reg_data_t reg_data, clk_data;

static void __raw_writel(uint32_t data, void *ptr) {
    debug("writing 0x%08x to 0x%08hx\n",data, ptr - otp_base);
    *((uint32_t*)ptr) = data;
}

static uint32_t __raw_readl(void *ptr) {
    debug("reading 0x%08x at 0x%08hx\n",*((uint32_t*)ptr), ptr - otp_base);
    return *((uint32_t*)ptr);
}

static uint32_t clk_get_rate() {
    uint32_t result, cbcdr, cbcmr;

    cbcdr = __raw_readl(ccm_base + HW_CCM_CBCDR);
    cbcmr = __raw_readl(ccm_base + HW_CCM_CBCMR);

    result = ccm_pre_periph_clk_sources[FB(cbcmr, CCM_CBCMR_PRE_PERIPH_CLK_SEL)];
    result /= FB(cbcdr, CCM_CBCDR_AHB_PODF)+1;
    result /= FB(cbcdr, CCM_CBCDR_IPG_PODF)+1;

    debug("clock rate is %zu\n", result);
    return result;
}


static int clk_set(int state) {
    int res = 0;
    uint32_t mask, value;
    mask = BM_CCM_CGR2_OCOTP;
    //WARNING: THIS IS DONE WITHOUT LOCKING, 
    //NO SCT REGISTER AND NO LOCKS AVAILABLE IN USER-SPACE
    value = __raw_readl(ccm_base+HW_CCM_CGR2);
    if(state) value |= mask;
    else value &= ~mask;
    __raw_writel(value, ccm_base+HW_CCM_CGR2);
    debug("clock state is %d at 0x%08x\n",state, value);
    return res;
}

static void set_otp_timing(void)
{
    uint32_t clk_rate = 0;
    uint32_t strobe_read, relex, strobe_prog;
    uint32_t timing = 0;

    clk_rate = clk_get_rate();

    /* do optimization for too many zeros */
    relex = clk_rate / (1000000000 / DEF_RELAX) - 1;
    strobe_prog = clk_rate / (1000000000 / 10000) + 2 * (DEF_RELAX + 1) - 1;
    strobe_read = clk_rate / (1000000000 / 40) + 2 * (DEF_RELAX + 1) - 1;

    timing = BF(relex, OCOTP_TIMING_RELAX);
    timing |= BF(strobe_read, OCOTP_TIMING_STROBE_READ);
    timing |= BF(strobe_prog, OCOTP_TIMING_STROBE_PROG);

    __raw_writel(timing, otp_base + HW_OCOTP_TIMING);
}

static int otp_wait_busy(uint32_t flags)
{
    int count;
    uint32_t c;

    for (count = 10000; count >= 0; count--) {
        c = __raw_readl(otp_base + HW_OCOTP_CTRL);
        if (!(c & (BM_OCOTP_CTRL_BUSY | BM_OCOTP_CTRL_ERROR | flags)))
            break;
        usleep(1);
    }

    if (count < 0)
        return -1;

    return 0;
}

int fsl_otp_readl(uint32_t offset, uint32_t *value)
{
    int ret = 0;

    ret = clk_set(1);
    if (ret)
        return ret;

    set_otp_timing();
    ret = otp_wait_busy(0);
    if (ret)
        goto out;

    *value = __raw_readl(otp_base + offset);

out:
    clk_set(0);
        return ret;
}

static int otp_write_bits(int addr, uint32_t data, uint32_t magic)
{
    uint32_t c; /* for control register */

    /* init the control register */
    c = __raw_readl(otp_base + HW_OCOTP_CTRL);
    c &= ~BM_OCOTP_CTRL_ADDR;
    c |= BF(addr, OCOTP_CTRL_ADDR);
    c |= BF(magic, OCOTP_CTRL_WR_UNLOCK);
    __raw_writel(c, otp_base + HW_OCOTP_CTRL);

    /* init the data register */
    __raw_writel(data, otp_base + HW_OCOTP_DATA);
    otp_wait_busy(0);

    usleep(2000); /* Write Postamble */

    return 0;
}

static int fsl_otp_store(int index, uint32_t value)
{
    int ret;

    ret = clk_set(1);
    if (ret)
        return 0;

    set_otp_timing();
    ret = otp_wait_busy(0);
    if (ret)
        goto out;

    otp_write_bits(index, value, 0x3e77);

    /* Reload all the shadow registers */
    __raw_writel(BM_OCOTP_CTRL_RELOAD_SHADOWS,
             otp_base + HW_OCOTP_CTRL_SET);
    usleep(1);
    otp_wait_busy(BM_OCOTP_CTRL_RELOAD_SHADOWS);

out:
    clk_set(0);
    return ret;
}

static int read_regdata(char *path, struct reg_data_t *reg_data) {
    int reg_fd, res = -1;

    reg_fd = open(path, O_RDONLY | O_SYNC);
    if(reg_fd) {
        res = read(reg_fd, reg_data, sizeof(struct reg_data_t));
        reg_data->addr = __bswap_32 (reg_data->addr);
        reg_data->size = __bswap_32 (reg_data->size);
        debug("register is located at 0x%04x with size 0x%04x\n",reg_data->addr, reg_data->size);
        close(reg_fd);
    }
    return res;
}

static int fsl_otp_probe(void)
{
    int ret, fd;

    if((ret = read_regdata(PROC_OCOTP_PATH, &reg_data)) < 1) {
        error("failed to verify ocotp address\n");
        return -1;
    }

    if((ret = read_regdata(PROC_CCM_PATH, &clk_data)) < 1) {
        error("failed to verify ccm address\n");
        return -1;
    }

    if ((fd = open ("/dev/mem", O_RDWR | O_SYNC) ) < 0) {
        error("Unable to open /dev/mem: %i\n", fd );
        return -1;
    }

    otp_base = mmap(0, reg_data.size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, reg_data.addr);
    if ((int32_t)otp_base < 0){
        error("mmap failed: 0x%08x\n", (int32_t)otp_base);
        return -1;
    }

    ccm_base = mmap(0, clk_data.size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, clk_data.addr);
    if ((int32_t)otp_base < 0){
        error("mmap failed: 0x%08x\n", (int32_t)otp_base);
        return -1;
    }

    close(fd);

    return 0;
}

static int trace_reg(const char *needle, const char **haystack, size_t size) {
    int i;
    for(i = 0; i < size; i++) {
        if(!strcmp(needle, haystack[i])) return i;
    }
    return -1;
}

static void error_clear() {
    uint32_t value;
    value = __raw_readl(otp_base + HW_OCOTP_CTRL);
    if (value & BM_OCOTP_CTRL_ERROR) {
        debug("Error bit was set, unsetting!!!!!\n");
        clk_set(1);
        __raw_writel(BM_OCOTP_CTRL_ERROR, otp_base + HW_OCOTP_CTRL_CLR);
        otp_wait_busy(0);
        clk_set(0);
    }
}

int main( int argc, const char **argv ) {
    const char **registers;
    int index, regsize, ret;
    uint32_t value, new_value;

    registers = &imx6q_otp_desc[0][0];
    regsize = sizeof(imx6q_otp_desc)/sizeof(char*);

    if((ret = fsl_otp_probe())) return ret;

    error_clear();

    if(argc != 2 && argc != 4 && !!strcmp("write", argv[2])) {
        printf("Usage %s <REGNAME> [write 0x<NEW_VALUE>]\nValid values for REGNAME are: \n%s", argv[0], registers[0]);
        for(index = 1; index < regsize; index++)
            printf(", %s", registers[index]);
        printf("\n\nWARNING: Use at your own risk, you can brick your hardware if you don't know what you're doing!!!\n\n");
        ret = -1;
        goto out;
    }

    index = trace_reg(argv[1], registers, regsize);
    if(index < 0) {
        error("Unknown register\n");
        ret = -2;
        goto out;
    }

    ret = fsl_otp_readl(HW_OCOTP_CUST_N(index), &value);

    if(argc == 4) {
        new_value = 0;
        if(!sscanf(argv[3], "0x%x", &new_value)) {
            error("could not read data.\n");
            ret = -3;
            goto out;
        }
        debug("writing action received 0x%08x => 0x%08x \n", value, new_value);

        if( (new_value | value) != new_value) {
            error("current value is not contained in new value.\n");
            ret = -4;
            goto out;
        }

        fsl_otp_store(index, new_value);
        fsl_otp_readl(HW_OCOTP_CUST_N(index), &value);
        debug("writing action complete 0x%08x == 0x%08x \n", value, new_value);
        if(value != new_value) {
            error("write mismatch!!\n");
            ret = -5;
            goto out;
        }
    }

    error_clear();

    printf("0x%08x\n", value);

out:
    munmap(otp_base, reg_data.size);
    munmap(ccm_base, clk_data.size);
    return ret;
}
