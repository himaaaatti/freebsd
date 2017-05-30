#include "pci_emul.h"

struct pci_devemu pci_de_nvme = {
    .pe_emu = "nvme",
};
PCI_EMUL_SET(pci_de_nvme);
