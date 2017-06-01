#include <stdio.h>
#include <stdlib.h>

#include <dev/nvme/nvme.h>

#include "pci_emul.h"

#define NVME_DEBUG

#ifdef NVME_DEBUG
static FILE *dbg;
#define DPRINTF(format, arg...)	do{fprintf(dbg, format, ##arg);fflush(dbg);}while(0)
#else
#define DPRINTF(format, arg...)
#endif

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
    pci_emul_alloc_bar(pi, 0, PCIBAR_MEM64, 0x1000);

	sc = calloc(1, sizeof(struct pci_nvme_softc));
	pi->pi_arg = sc;
/*     sc->asc_pi = pi; */

    sc->regs.cap_hi.raw = 0x00000000;
    sc->regs.cap_lo.raw = 0x02000000;

    nvme_reset(sc);

    return 0;
}

static void
pci_nvme_write(struct vmctx *ctx, int vcpu, struct pci_devinst *pi,
		int baridx, uint64_t offset, int size, uint64_t value)
{

}

static uint64_t
pci_nvme_read(struct vmctx *ctx, int vcpu, struct pci_devinst *pi, int baridx,
    uint64_t regoff, int size)
{

    DPRINTF("0x%lx",  regoff);
    return 0;
}

struct pci_devemu pci_de_nvme = {
    .pe_emu = "nvme",
    .pe_init = pci_nvme_init,
    .pe_barwrite = pci_nvme_write,
    .pe_barread = pci_nvme_read
};
PCI_EMUL_SET(pci_de_nvme);
