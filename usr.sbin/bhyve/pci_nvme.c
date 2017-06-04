#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <assert.h>
#include <pthread.h>

#include <machine/vmm.h>
#include <vmmapi.h>

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
    NVME_CR_CSTS    = 0x1c,
    NVME_CR_NSSR    = 0x20,
    NVME_CR_AQA     = 0x24,
    NVME_CR_ASQ_LOW = 0x28,
    NVME_CR_ASQ_HI  = 0x2c,
    NVME_CR_ACQ_LOW = 0x30,
    NVME_CR_ACQ_HI  = 0x34,
/*     NVME_CR_CMBLOC  = 0x38, */ // optional
/*     NVME_CR_CMBSZ   = 0x3c, */
/*     NVME_CR_BPINFO  = 0x40, */
/*     NVME_CR_BPRSEL  = 0x44, */
/*     NVME_CR_BPMBL   = 0x48, */
    NVME_CR_IO_QUEUE_BASE = 0x1000, // submission queue 0 tail doorbell (admin)
};

struct pci_nvme_softc {
    struct nvme_registers regs;
    uintptr_t asq_base;
    uintptr_t acq_base;
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
    if(pci_emul_alloc_bar(pi, 0, PCIBAR_MEM64, 0x100f))
    {
        DPRINTF("error is occured in pci_emul_alloc_bar\n");
        return 1;
    }
    if(pci_emul_add_msixcap(pi, 4, 4)) //XXX fix msix num 
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

static void
pci_nvme_setup_controller(struct vmctx *ctx, struct pci_nvme_softc *sc)
{
    //TODO
    sc->regs.aqa.bits.acqs;
    sc->asq_base = (uintptr_t)vm_map_gpa(ctx, sc->regs.asq, sc->regs.aqa.bits.asqs);
    sc->acq_base = (uintptr_t)vm_map_gpa(ctx, sc->regs.acq, sc->regs.aqa.bits.acqs);

    sc->regs.csts.bits.rdy = 1;
}

static void 
pci_nvme_write_bar_0(struct vmctx* ctx, struct pci_nvme_softc *sc, uint64_t regoff, uint64_t value, int size)
{

    // write value to doorbell register
    if(regoff >= NVME_CR_IO_QUEUE_BASE && regoff <= 0x100f)
    {
        int offset = regoff - 0x1000;
        // submission queue
        if(!(offset % 2))
        {
            //TODO
            // admin queue
            if((offset / 8) == 0)
            {
            }
        }
        // completion queue
        else 
        {
            //TODO
            // admin queue
            if((offset / 8) == 0)
            {
            }
        }
        assert(0);
        return;
    }

    switch(regoff) 
    {
        case NVME_CR_CC:
            if(size == 4)
            {
                //TODO 
                // - 1 -> 0
                //  Controler Reset
                //      - delete all I/O Submission queues and Completion queues
                //      - reset admin submittion queue and completion queue
                //      - move to idle state
                //      - all controller registers are reset to their default values (defined in section 5.21.1)
                //       
                //      - CSTS.RDY bit is cleared to '0'.
                //
                // - 0 -> 1
                //      - When a controller is ready to process commands, the controller set '1' to CSTS.RDY.
                //

                if((sc->regs.cc.bits.en == 0) && (value & 0x1)) 
                {
                    pci_nvme_setup_controller(ctx, sc);
                }

                if((sc->regs.cc.bits.en == 1) && !(value & 0x1))
                {
                    sc->regs.csts.bits.rdy = 0;
                    //TODO
                    assert(0);
                }

                sc->regs.cc.raw = (uint32_t)value;
                return;
            }
            else 
            {
                assert(0);
            }
        case NVME_CR_AQA:
            if(size == 4)
            {
                sc->regs.aqa.raw = (uint32_t)value;
                return;
            }
            else
            {
                assert(0);
            }
        case NVME_CR_ASQ_LOW:
            if(size == 4)
            {
                sc->regs.asq = (sc->regs.asq & 0xffffffff00000000) | value;
                return;
            }
            else 
            {
                assert(0);
            }
        case NVME_CR_ASQ_HI:
            if(size == 4)
            {
                sc->regs.asq = (sc->regs.asq & 0x00000000ffffffff) | ((0xfffff000 & value) << 32);
                return;
            }
            else {
                assert(0);
            }
        case NVME_CR_ACQ_LOW:
            if(size == 4)
            {
                sc->regs.acq = (sc->regs.acq & 0xffffffff00000000) | value;
                return;
            }
            else 
            {
                assert(0);
            }

        case NVME_CR_ACQ_HI:
            if(size == 4)
            {
                sc->regs.acq = (sc->regs.acq & 0x00000000ffffffff) | (value << 32);
                return;
            }
            else 
            {
                assert(0);
            }
        default:
            DPRINTF("unknown regoff 0x%lx with value 0x%lx, size %d in %s\n", regoff, value, size, __func__);
            assert(0);
    }
}

static void
pci_nvme_write(struct vmctx *ctx, int vcpu, struct pci_devinst *pi,
		int baridx, uint64_t regoff, int size, uint64_t value)
{
    struct pci_nvme_softc *sc = pi->pi_arg;

    if(baridx == pci_msix_table_bar(pi) || 
            baridx == pci_msix_pba_bar(pi))
    {
        DPRINTF("msix: regoff 0x%lx, size %d, value %lx\n", regoff, size, value);
        pci_emul_msix_twrite(pi, regoff, size, value);
        return;
    }
    DPRINTF("write regoff 0x%lx, size %d, value %lx\n", regoff, size, value);

    switch(baridx)
    {
        case 0:
            pci_nvme_write_bar_0(ctx, sc, regoff, value, size);
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
                DPRINTF("not implemented, regoff 0x%lx, size %d\n", regoff, size);
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

        case NVME_CR_CSTS:
            if(size == 4)
            {
                return (uint64_t)sc->regs.csts.raw;
            }
            else 
            {
                assert(0);
            }

        default:
            DPRINTF("unknown regoff value: 0x%lx, size %d in %s\n", regoff, size, __func__);
            assert(0);
    }

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
        DPRINTF("msix: regoff 0x%lx, size %d\n", regoff, size);
        return pci_emul_msix_tread(pi, regoff, size);
    }
    DPRINTF("read regoff 0x%lx, size %d\n", regoff, size);

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
