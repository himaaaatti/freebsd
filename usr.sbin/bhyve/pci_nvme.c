#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#include <dev/nvme/nvme.h>

#include "pci_emul.h"

#define NVME_DEBUG

#ifdef NVME_DEBUG
static FILE *dbg;
#define DPRINTF(format, arg...)	do{fprintf(dbg, format, ##arg);fflush(dbg);}while(0)
#else
#define DPRINTF(format, arg...)
#endif

enum nvme_controller_register_offsets {
    NVME_CR_CAP_LOW = 0x00,
    NVME_CR_CAP_HI  = 0x04,
    NVME_CR_VS      = 0x08,
    NVME_CR_INTMS   = 0x0c,
    NVME_CR_INTMC   = 0x10,
    NVME_CR_CC      = 0x14,
    NVME_CR_CSTS    = 0x10,
    NVME_CR_NSSR    = 0x20,
    NVME_CR_AQA     = 0x24,
    NVME_CR_ASQ     = 0x28,
    NVME_CR_ACQ     = 0x30,
    NVME_CR_CMBLOC  = 0x38,
    NVME_CR_CMBSZ   = 0x3c,
    NVME_CR_BPINFO  = 0x40,
    NVME_CR_BPRSEL  = 0x44,
    NVME_CR_BPMBL   = 0x48,
    NVME_CR_IO_QUEUE_BASE = 0x1000, // submission queue 0 tail doorbell (admin)
};

struct pci_nvme_softc {
    struct nvme_registers regs;
};

static void
nvme_reset(struct pci_nvme_softc *sc)
{
/*     sc->nssr = 0x4e564d65; */
}

static int 
pci_nvme_init (struct vmctx *ctx, struct pci_devinst *pi, char *opts)
{
    struct pci_nvme_softc *sc;

#ifdef NVME_DEBUG
	dbg = fopen("/tmp/nvme_emu_log", "w+");
#endif

    pci_set_cfgdata16(pi, PCIR_DEVICE, 0x0111);
    pci_set_cfgdata16(pi, PCIR_VENDOR, 0x8086);
    // for NVMe Controller Registers
    if(pci_emul_alloc_bar(pi, 0, PCIBAR_MEM64, 0x1000))
    {
        DPRINTF("error is occured in pci_emul_alloc_bar\n");
        return 1;
    }
    if(pci_emul_add_msixcap(pi, 2, 4)) //XXX fix msix num 
    {
        DPRINTF("error is occured in pci_emul_add_msixcap\n");
        return 1;
    }

	sc = calloc(1, sizeof(struct pci_nvme_softc));
	pi->pi_arg = sc;
/*     sc->asc_pi = pi; */

    sc->regs.cap_hi.raw = 0x00000000;
    sc->regs.cap_lo.raw = 0x02000000;

    nvme_reset(sc);

    return 0;
}


static uint64_t
pci_nvme_write_bar_0(struct pci_nvme_softc *sc, uint64_t offset, uint64_t value, int size)
{
    switch(offset) 
    {
        default:
            DPRINTF("unknown offset 0x%lx with 0x%lx in %s\n", offset, value, __func__);
            assert(0);
    }
}

static uint64_t
pci_nvme_write_bar_4(struct pci_nvme_softc *sc, uint64_t offset, uint64_t value, int size)
{
    switch(offset)
    {
        default:
            DPRINTF("unknown offset 0x%lx with 0x%lx in %s\n", offset, value, __func__);
            assert(0);
    }
}


static void
pci_nvme_write(struct vmctx *ctx, int vcpu, struct pci_devinst *pi,
		int baridx, uint64_t offset, int size, uint64_t value)
{
    struct pci_nvme_softc *sc = pi->pi_arg;

    if(baridx == pci_msix_table_bar(pi) || 
            baridx == pci_msix_pba_bar(pi))
    {
        pci_emul_msix_twrite(pi, offset, size, value);
        return;
    }


    switch(baridx)
    {
        case 0:
            pci_nvme_write_bar_0(sc, offset, value, size);
            break;

        default:
            DPRINTF("unknown baridx %d with 0x%lx in %s\n", baridx, value, __func__);
            assert(0);
    }
}

static uint64_t
pci_nvme_read_bar_0(struct pci_nvme_softc *sc, uint64_t regoff, int size)
{
    switch (regoff) {
        case NVME_CR_CAP_LOW:
            if(size == 4) 
            {
                return (uint64_t)sc->regs.cap_lo.raw;
            }
            else {
                DPRINTF("not implemented, offset 0x%lx, size %d\n", regoff, size);
                assert(0);
            }


        case NVME_CR_CAP_HI:
            if(size == 4)
            {
                return (uint64_t)sc->regs.cap_hi.raw;
            }
            else {
                assert(0);
            }

       case NVME_CR_CC:
            if(size == 4)
            {
                return (uint64_t)sc->regs.cc.raw;
            }
            else 
            {
                assert(0);
            }

        default:
            DPRINTF("unknown offset value: 0x%lx in %s\n", regoff, __func__);
            assert(0);
    }

    return 0;
}

static uint64_t
pci_nvme_read_bar_4(struct pci_nvme_softc *sc, uint64_t regoff, int size)
{
    assert(0);
    return 0;
}

static uint64_t
pci_nvme_read(struct vmctx *ctx, int vcpu, struct pci_devinst *pi, int baridx,
    uint64_t regoff, int size)
{
    struct pci_nvme_softc *sc = pi->pi_arg;

    if(baridx == pci_msix_table_bar(pi) ||
            baridx == pci_msix_pba_bar(pi))
    {
        return pci_emul_msix_tread(pi, regoff, size);
    }

    switch (baridx) {
        case 0:
            return pci_nvme_read_bar_0(sc, regoff, size);

        default:
            DPRINTF("unknown bar %d, 0x%lx\n", baridx, regoff);
            assert(0);
    }

    return 0;
}

struct pci_devemu pci_de_nvme = {
    .pe_emu = "nvme",
    .pe_init = pci_nvme_init,
    .pe_barwrite = pci_nvme_write,
    .pe_barread = pci_nvme_read
};
PCI_EMUL_SET(pci_de_nvme);
