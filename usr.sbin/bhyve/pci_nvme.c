#include <stdio.h>
#include <stdlib.h>

#include "pci_emul.h"
#include "block_if.h"

#include <dev/nvme/nvme.h>

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

#define MAX_CQ_NUM 4
#define MAX_SQ_NUM 4

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
     * - 0x100c ~ 0x100f
     *  Completion queue tail doorbell register for nvm commands
     *
     * */

    /*     status = pci_emul_alloc_bar(pi, 0, PCIBAR_MEM64, 0x100f); */
    status = pci_emul_alloc_bar(pi, 0, PCIBAR_MEM32, 0x100f);
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
pci_nvme_write(struct vmctx* ctx, int vcpu, struct pci_devinst* pi, int baridx,
                           uint64_t offset, int size, uint64_t value)
{
    assert(0);
}

static uint64_t
pci_nvme_read(struct vmctx *ctx, int vcpu, struct pci_devinst *pi, int baridx,
    uint64_t regoff, int size)
{
    assert(0);
}


struct pci_devemu pci_de_nvme = {
    .pe_emu = "nvme",
    .pe_init = pci_nvme_init,
    .pe_barwrite = pci_nvme_write,
    .pe_barread = pci_nvme_read,

};
PCI_EMUL_SET(pci_de_nvme);
