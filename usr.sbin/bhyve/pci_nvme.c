
#include "pci_emul.h"

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

static int 
pci_nvme_init(struct vmctx* ctx, struct pci_devinst* pi, char* ops)
{
    assert(0);
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
