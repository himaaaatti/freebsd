#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include <dev/nvme/nvme.h>

#include "pci_emul.h"
#include "block_if.h"
#include "bhyverun.h"

#ifdef NVME_DEBUG
static FILE* dbg;
#define DPRINTF(format, arg...)      \
    do {                             \
        fprintf(dbg, format, ##arg); \
        fflush(dbg);                 \
    } while (0)
#else
#define DPRINTF(format, arg...)
#endif

#define ctx_from_sc(sc) ((sc)->pi->pi_vmctx)

enum nvme_controller_register_offsets {
    NVME_CR_CAP_LOW = 0x00,
    NVME_CR_CAP_HI = 0x04,
    NVME_CR_VS = 0x08,
    NVME_CR_INTMS = 0x0c,
    NVME_CR_INTMC = 0x10,
    NVME_CR_CC = 0x14,
    NVME_CR_CSTS = 0x1c,
    NVME_CR_NSSR = 0x20,
    NVME_CR_AQA = 0x24,
    NVME_CR_ASQ_LOW = 0x28,
    NVME_CR_ASQ_HI = 0x2c,
    NVME_CR_ACQ_LOW = 0x30,
    NVME_CR_ACQ_HI = 0x34,
    // submission queue 0 tail doorbell (admin)
    NVME_CR_IO_QUEUE_BASE = 0x1000,
};

enum nvme_cc_bits {
    NVME_CC_EN = 0x00000001,
    NVME_CC_RSV0 = 0x0000000e,
    NVME_CC_CCS = 0x00000070,
    NVME_CC_MPS = 0x00000780,
    NVME_CC_AMS = 0x00003800,
    NVME_CC_SHN = 0x0000c000,
    NVME_CC_IOSQES = 0x000f0000,
    NVME_CC_IOCQES = 0x00f00000,
    NVME_CC_RSV1 = 0xff000000,
};

#ifdef NVME_DEBUG
const char* get_nvme_cr_text(enum nvme_controller_register_offsets offset,
                             bool is_read)
{
    switch (offset) {
        case NVME_CR_CAP_LOW:
            return "CAP_LOW";
        case NVME_CR_CAP_HI:
            return "CAP_HI";
        case NVME_CR_VS:
            return "VS";
        case NVME_CR_CC:
            return "CC";
        case NVME_CR_CSTS:
            return "CSTS";
        case NVME_CR_AQA:
            return "AQA";
        case NVME_CR_ASQ_LOW:
            return "ASQ_LOW";
        case NVME_CR_ASQ_HI:
            return "ASQ_HI";
        case NVME_CR_ACQ_LOW:
            return "ACQ_LOQ";
        case NVME_CR_ACQ_HI:
            return "ACQ_HI";

        default:
            if (is_read) {
                DPRINTF("read ");
            }
            else {
                DPRINTF("write ");
            }
            DPRINTF("0x%x\n", offset);
            assert(0);
    }
}
#endif

#define MAX_CQ_NUM 4
#define MAX_SQ_NUM 4
#define DOORBELL_LIMIT 0x100f

struct nvme_features {
    union {
        uint32_t raw;
        struct {
            uint16_t over;
            uint16_t under;
        } __packed bits;
    } __packed temparture_threshold;

    union {
        uint32_t raw;
        struct {
            uint8_t thr : 8;
            uint8_t time : 8;
            uint16_t reserved : 16;
        } __packed bits;
    } __packed interrupt_coalscing;
};

struct nvme_completion_queue {
    uintptr_t base_addr;
    uint16_t interrupt_vector;
    uint16_t size;
    uint16_t qid;
};

struct nvme_submission_queue {
    uintptr_t base_addr;
    uint16_t size;
    uint16_t qid;
    uint16_t completion_qid;
    //XX ioreq
};

struct pci_nvme_softc {
    struct pci_devinst* pi;
    struct blockif_ctxt* bctx;
    struct nvme_registers regs;
    uint16_t submission_queue_head;
    uint16_t completion_queue_tail;
    struct nvme_completion_queue* cqs;
    struct nvme_submission_queue* sqs;
    struct nvme_controller_data controller_data;
    struct nvme_namespace_data namespace_data;
    struct nvme_features features;
};

static void 
pci_nvme_softc_reset(struct pci_nvme_softc* sc)
{
    /*
     * Initialize NVMe controller register values.
     * this initializing according to NVMe specification 1.1b.
     */
    sc->regs.cap_lo.bits.mqes = 0x10;
    sc->regs.cap_lo.bits.cqr = 1;
    sc->regs.cap_lo.bits.ams = 0;
    sc->regs.cap_lo.bits.reserved1 = 0;
    sc->regs.cap_lo.bits.to = 10;

    sc->regs.cap_hi.bits.dstrd = 0;
    sc->regs.cap_hi.bits.reserved3 = 0;
    sc->regs.cap_hi.bits.css_nvm = 0;
    sc->regs.cap_hi.bits.css_reserved = 0;
    sc->regs.cap_hi.bits.reserved2 = 0;
    sc->regs.cap_hi.bits.mpsmin = 0;
    sc->regs.cap_hi.bits.mpsmax = 0;
    sc->regs.cap_hi.bits.reserved1 = 0;

    uint32_t version = (0x0001 << 16) | 0x0000;
    sc->regs.vs = version;

    sc->regs.cc.raw = 0;

    sc->regs.csts.raw = 0;

    sc->regs.asq = 0;
    sc->regs.acq = 0;

    sc->submission_queue_head = 0;
    sc->completion_queue_tail = 0;

    /*
     * Initialize identify data
     */

    /* number of namespace is 1 */
    sc->controller_data.nn = 0x1;

    /* Logic block format */
    sc->namespace_data.lbaf[0].ms = 0x0;
    /*
     * LB data size must be 2^n. (n is started by 0)
     * should be larger than 9. (i.e 512 bytes)
     */
    int lba_data_size = 9;
    sc->namespace_data.lbaf[0].lbads = lba_data_size;
    sc->namespace_data.lbaf[0].rp = 0x0;
    sc->namespace_data.nlbaf = 0x00;
    sc->namespace_data.flbas.format = 0x0;
    sc->namespace_data.nlbaf = 0x1;

    int block_size = blockif_size(sc->bctx);
    sc->namespace_data.nsze = block_size / (2 << (lba_data_size - 1));
    sc->namespace_data.ncap = block_size / (2 << (lba_data_size - 1));

    /*
     * Initlize features
     */
    sc->features.temparture_threshold.bits.over = 0xffff;
    sc->features.temparture_threshold.bits.under = 0x0000;

    sc->features.interrupt_coalscing.raw = 0;
}

static int 
pci_nvme_init(struct vmctx* ctx, struct pci_devinst* pi, char* opts)
{
    struct pci_nvme_softc* sc;
    struct blockif_ctxt* bctx;
    int status;

#ifdef NVME_DEBUG
    dbg = fopen("/tmp/nvme_emul_log", "w+");
    DPRINTF("----- initialize nvme controller emulator -----\n");
#endif

    if (opts == NULL) {
        fprintf(stderr, "pci_nvme: backing device required");
        return 1;
    }
    DPRINTF("backing file is %s\n", opts);

    /* Configulation for pci */
/*     pci_set_cfgdata16(pi, PCIR_DEVICE, 0x0953); */
    pci_set_cfgdata16(pi, PCIR_DEVICE, 0x0111);
    pci_set_cfgdata16(pi, PCIR_VENDOR, 0x8086);
    pci_set_cfgdata8(pi, PCIR_CLASS, PCIC_STORAGE);

    /*
     * NVMe Controller Registers are located in the PCI BAR0 and BAR1
     * - 0x0000 ~ 0x0fff
     *  NVMe Controller Registers
     *  Details are in NVM Express specification
     *
     * - 0x1000 ~ 0x1003
     *  Submission queue tail doorbell register for admin commands
     * - 0x1004 ~ 0x1007
     *  Completion queue head doorbell register for admin commands
     * - 0x1008 ~ 0x100b
     *  Submission queue tail doorbell register for nvm commands
     * - 0x100c ~ DOORBELL_LIMIT(0x100f)
     *  Completion queue tail doorbell register for nvm commands
     *
     * */

    /*     status = pci_emul_alloc_bar(pi, 0, PCIBAR_MEM64, 0x100f); */
    status = pci_emul_alloc_bar(pi, 0, PCIBAR_MEM32, DOORBELL_LIMIT);
    if (status) {
        DPRINTF("Error is occurred in pci_emul_alloc_bar\n");
        return 1;
    }

    status = pci_emul_add_msixcap(pi, 4, 2);
    if (status) {
        DPRINTF("Error is occurred in pci_emul_add_msixcap\n");
        return 1;
    }
    DPRINTF("msix table bar %d, pda bar %d\n", pci_msix_table_bar(pi),
            pci_msix_pba_bar(pi));

    sc = calloc(1, sizeof(struct pci_nvme_softc));
    pi->pi_arg = sc;
    pi->pi_vmctx = ctx;
    sc->pi = pi;

    bctx = blockif_open(opts, "");
    if (bctx == NULL) {
        DPRINTF("backing file (%s) couldn't open\n", opts);
        goto free_sc;
    }

    sc->bctx = bctx;

    pci_nvme_softc_reset(sc);

    sc->cqs = calloc(MAX_CQ_NUM, sizeof(struct nvme_completion_queue));
    sc->sqs = calloc(MAX_SQ_NUM, sizeof(struct nvme_submission_queue));

    //TODO: Initialize Submission queue

    return 0;

free_sc:
    free(sc);
    return 1;
}

static void
pci_nvme_setup_controller(struct pci_nvme_softc* sc)
{
    sc->regs.asq =
        (uintptr_t)paddr_guest2host(ctx_from_sc(sc), sc->regs.asq,
                sizeof(struct nvme_command) * sc->regs.aqa.bits.asqs);
    sc->regs.acq = 
        (uintptr_t)paddr_guest2host(ctx_from_sc(sc), sc->regs.acq,
                sizeof(struct nvme_completion) * sc->regs.aqa.bits.acqs);
}

static void
pci_nvme_write_bar_0(struct vmctx* ctx, struct pci_nvme_softc* sc,
        uint64_t offset, uint64_t value, int size)
{
    /*
     * Guest write to doorbell registers
     * - offset
     *  0x1000 ~ DOORBELL_LIMIT(0x100f)
     */
    if (offset >= NVME_CR_IO_QUEUE_BASE && offset <= DOORBELL_LIMIT) {
        //TODO
        assert(0);
    }

    /*
     * Guest access to NVMe controller reigsters
     * - offset
     *   0x0000 ~ 0x1000
     */
    assert(size == 4 && "word size should be 4 (byte)");
    switch (offset){
        case NVME_CR_CC:
            /*
             * - 00 Enable
             *   - 1 -> 0 Controller Reset
             *      //TODO
             *   - 0 -> 1 Start
             *      Controller setup to process command. When controler 
             *      finished preparation, controller arrise CSTS.RDY bit.
             */
            if(!sc->regs.cc.bits.en && (value & NVME_CC_EN)) {
                pci_nvme_softc_reset(sc);
                pci_nvme_setup_controller(sc);
            }
            if(sc->regs.cc.bits.en && !(value & NVME_CC_EN)) {
                sc->regs.csts.bits.rdy = 0;
            }

            /*
             * Processing for shutdown
             */
            switch((value & NVME_CC_SHN) >> 14) {
                case 0:
                    break;
                case NVME_SHN_NORMAL:
                    sc->regs.csts.bits.shst = NVME_SHST_COMPLETE;
                case NVME_SHN_ABRUPT:
                default:
                    assert(0 && "not yet implemented");
            }

            sc->regs.cc.raw = (uint32_t)value;
            return;

        case NVME_CR_AQA:
            sc->regs.aqa.raw = (uint32_t)value;
            return;

        case NVME_CR_ASQ_LOW:
            sc->regs.asq =
                (sc->regs.asq & 0xffffffff00000000) | (0xfffff000 & value);
            return;

        case NVME_CR_ASQ_HI:
            sc->regs.asq = (sc->regs.asq & 0x00000000ffffffff) | (value << 32);
            return;

        case NVME_CR_ACQ_LOW:
            sc->regs.acq =
                (sc->regs.acq & 0xffffffff00000000) | (0xfffff000 & value);
            return;

        case NVME_CR_ACQ_HI:
            sc->regs.acq = (sc->regs.acq & 0x00000000ffffffff) | (value << 32);
            return;

        default:
            DPRINTF("a behavior of the registers is not yet implemented.\n");
            DPRINTF("offset 0x%lx, value 0x%lx\n", offset, value);
            assert(0);
    }
}

static void 
pci_nvme_write(struct vmctx* ctx, int vcpu, struct pci_devinst* pi, int baridx,
                           uint64_t offset, int size, uint64_t value)
{

    struct pci_nvme_softc *sc = pi->pi_arg;

    if(baridx == pci_msix_table_bar(pi) || baridx == pci_msix_table_bar(pi)) {
        pci_emul_msix_twrite(pi, offset, size, value);
        return;
    }

    switch (baridx) {
        case 0:
            pci_nvme_write_bar_0(ctx, sc, offset, value, size);
        default:
            DPRINTF("unknown regoff value: 0x%lx, size %d in %s\n", offset, size,
                    __func__);
            assert(0);
    }
}

static uint64_t 
pci_nvme_read_bar_0(struct pci_nvme_softc* sc, uint64_t offset, int size)
{
    DPRINTF("read %s\n", get_nvme_cr_text(offset, true));
    assert(size == 4 && "word size should be 4.");
    switch (offset) {
        case NVME_CR_CAP_LOW:
            return (uint64_t)sc->regs.cap_lo.raw;

        case NVME_CR_CAP_HI:
            return (uint64_t)sc->regs.cap_hi.raw;

        case NVME_CR_VS:
            return (uint64_t)sc->regs.vs;

        case NVME_CR_CC:
            return (uint64_t)sc->regs.cc.raw;

        case NVME_CR_CSTS:
            DPRINTF("CSTS raw 0x%x\n", sc->regs.csts.raw);
            return (uint64_t)sc->regs.csts.raw;

        default:
            DPRINTF("unknown regoff value: 0x%lx, size %d in %s\n", offset, size,
                    __func__);
            assert(0);
    }
}

static uint64_t
pci_nvme_read(struct vmctx *ctx, int vcpu, struct pci_devinst *pi, int baridx,
    uint64_t regoff, int size)
{
    struct pci_nvme_softc* sc = pi->pi_arg;
    if (baridx == pci_msix_table_bar(pi) || baridx == pci_msix_table_bar(pi)) {
        return pci_emul_msix_tread(pi, regoff, size);
    }

    switch (baridx) {
        case 0:
            return pci_nvme_read_bar_0(sc, regoff, size);

        default:
            assert(0);
    }
}

struct pci_devemu pci_de_nvme = {
    .pe_emu = "nvme",
    .pe_init = pci_nvme_init,
    .pe_barwrite = pci_nvme_write,
    .pe_barread = pci_nvme_read,
};
PCI_EMUL_SET(pci_de_nvme);
