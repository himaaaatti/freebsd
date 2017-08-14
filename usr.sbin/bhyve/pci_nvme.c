#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <pthread.h>

#include <machine/vmm.h>
#include <vmmapi.h>

#include <dev/nvme/nvme.h>

#include "pci_emul.h"
#include "block_if.h"
#include "bhyverun.h"

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

    NVME_CR_IO_QUEUE_BASE = 0x1000, // submission queue 0 tail doorbell (admin)
};

const char* get_nvme_cr_text(enum nvme_controller_register_offsets offset)
{
    switch(offset)
    {
        case NVME_CR_CAP_LOW:
            return "CAP_LOW";
        case NVME_CR_CAP_HI:
            return "CAP_HI";
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
            DPRINTF("read 0x%x\n", offset);
            assert(0);
    }
}

const char* get_admin_command_text(enum nvme_admin_opcode opc)
{
    switch(opc) 
    {
        case NVME_OPC_DELETE_IO_SQ:
            return "delete i/o submission queue";
        case NVME_OPC_CREATE_IO_SQ:
            return "create i/o submission queue";
        case NVME_OPC_GET_LOG_PAGE:
            return "get log page";
        case NVME_OPC_DELETE_IO_CQ:
            return "delete i/o completion queue";
        case NVME_OPC_CREATE_IO_CQ:
            return "create i/o completion queue";
        case NVME_OPC_IDENTIFY:
            return "identify";
        case NVME_OPC_ABORT:
            return "abort";
        case NVME_OPC_SET_FEATURES:
            return "set feature";
        case NVME_OPC_GET_FEATURES:
            return "get feature";
        case NVME_OPC_ASYNC_EVENT_REQUEST:
            return "async event request";
        case NVME_OPC_FIRMWARE_ACTIVATE:
            return "firmware activate";
        case NVME_OPC_FIRMWARE_IMAGE_DOWNLOAD:
            return "firmware image download";
        case NVME_OPC_FORMAT_NVM:
            return "format nvm";
        case NVME_OPC_SECURITY_SEND:
            return "security send";
        case NVME_OPC_SECURITY_RECEIVE:
            return "security receive";

        default:
            assert(0 && "unknown opc\n") ;
    }
}

const char* get_nvm_command_text(enum nvme_nvm_opcode opc)
{
    switch(opc)
    {
        case NVME_OPC_FLUSH:
            return "flush";
        case NVME_OPC_WRITE:
            return "write";
        case NVME_OPC_READ:
            return "read";
        case NVME_OPC_WRITE_UNCORRECTABLE:
            return "write uncorrectable";
        case NVME_OPC_COMPARE:
            return "compare";
        case NVME_OPC_DATASET_MANAGEMENT:
            return "dataset management";

        default: 
            assert(0 && "unknown opc");
    }
}

enum nvme_cmd_identify_cdw10 {
    NVME_CMD_IDENTIFY_CDW10_CNTID   = 0xffff0000,
    NVME_CMD_IDENTIFY_CDW10_RSV     = 0x0000ff00,
    NVME_CMD_IDENTIFY_CDW10_CNS     = 0x000000ff,
};

enum nvme_cmd_identify_data {
    NVME_CMD_IDENTIFY_CNS_NAMESPACE     = 0x0,
    NVME_CMD_IDENTIFY_CNS_CONTROLLER    = 0x1,
};

const char* get_identify_command_type(enum nvme_cmd_identify_data cns)
{
    switch(cns)
    {
        case NVME_CMD_IDENTIFY_CNS_NAMESPACE:
            return "namespace";
        case NVME_CMD_IDENTIFY_CNS_CONTROLLER:
            return "controller";
        default:
            assert(0 && "unknown cns values");
    }
}


enum nvme_cc_bits {
    NVME_CC_EN      = 0x00000001,
    NVME_CC_RSV0    = 0x0000000e,
    NVME_CC_CCS     = 0x00000070,
    NVME_CC_MPS     = 0x00000780,
    NVME_CC_AMS     = 0x00003800,
    NVME_CC_SHN     = 0x0000c000,
    NVME_CC_IOSQES  = 0x000f0000,
    NVME_CC_IOCQES  = 0x00f00000,
    NVME_CC_RSV1    = 0xff000000,
};

struct nvme_features {
    union {
        uint32_t raw;
        struct {
            uint16_t ncqr   :16;
            uint16_t nsqr   :16;
        } __packed bits;
    } __packed num_of_queues;

    union {
        uint32_t raw;
        struct {
            uint16_t smart          :8;
            uint16_t ns_attr_noti   :1;
            uint16_t fw_act_noti    :1;
            uint16_t tele_log_noti  :1;
            uint32_t reserved       :21; 
        } __packed bits;
    } __packed async_event_config;

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
            uint8_t thr         :8;
            uint8_t time        :8;
            uint16_t reserved   :16;
        } __packed bits;
    } __packed interrupt_coalscing;
};

struct nvme_completion_queue_info {
    uintptr_t base_addr;
    uint16_t size;
    uint16_t head;
    uint16_t qid;
    pthread_mutex_t	mtx;
};

struct pci_nvme_softc;
// io request to block if.
struct nvme_ioreq {
    struct blockif_req io_req;
    struct nvme_completion completion_entry;
    struct nvme_completion_queue_info* cq_info;
    struct nvme_submission_queue_info* sq_info;
    struct pci_nvme_softc *sc;
    STAILQ_ENTRY(nvme_ioreq) io_flist;
    TAILQ_ENTRY(nvme_ioreq) io_blist;
};

struct nvme_submission_queue_info {
    uintptr_t base_addr;
    uint16_t size;
    uint16_t qid;
    uint16_t completion_qid;
    struct nvme_ioreq *ioreq;
    STAILQ_HEAD(nvme_fhead, nvme_ioreq) iofhd;
	TAILQ_HEAD(nvme_bhead, nvme_ioreq) iobhd;
};

/* struct nvme_namespace { */
/*     struct nvme_ioreq* ioreq; */
/*     struct blockif_ctxt *b_ctxt; */
/*     uint32_t nsid; */
/* }; */

struct pci_nvme_softc {
    struct nvme_registers regs;
    struct nvme_features features;
    struct pci_devinst *pi;
    uint16_t completion_queue_head;
    uint16_t submission_queue_tail;
    uintptr_t asq_base;
    uintptr_t acq_base;
    struct nvme_controller_data controller_data;
    struct nvme_namespace_data namespace_data;
    struct nvme_completion_queue_info *cqs_info;
    struct nvme_submission_queue_info *sqs_info;
    // TODO: consider about below varibale
    // How many do we need?
    struct blockif_ctxt *bctx;
};

static void
pci_nvme_reset(struct pci_nvme_softc *sc)
{
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

    sc->regs.cc.raw = 0;

    sc->regs.csts.raw = 0;

    sc->completion_queue_head = 0;
    sc->asq_base = 0;
    sc->acq_base = 0;
}

static void
initialize_feature(struct pci_nvme_softc *sc)
{
    sc->features.temparture_threshold.bits.over = 0xffff;
    sc->features.temparture_threshold.bits.under = 0x0000;

    sc->features.interrupt_coalscing.raw = 0;
    //TODO initialize other values
}

static void
initialize_identify(struct pci_nvme_softc *sc)
{
    sc->controller_data.nn = 0x2; // TODO consider this value

    // LBA format
    sc->namespace_data.lbaf[0].ms = 0x00;
    /*
     * LBA data size is 2^n (n is started by 0)
     * should be larger than 9.(i.e. 512 bytes)
     */
    uint64_t lba_data_size = 9;
    sc->namespace_data.lbaf[0].lbads = lba_data_size; 
    sc->namespace_data.lbaf[0].rp = 0x0;

    sc->namespace_data.nlbaf = 0x00;
    sc->namespace_data.flbas.format = 0x00;
    sc->namespace_data.nlbaf = 0x1;

    uint64_t block_size = blockif_size(sc->bctx);
    sc->namespace_data.nsze = block_size / (2 << (lba_data_size - 1));
    sc->namespace_data.ncap = block_size / (2 << (lba_data_size - 1));
}

static int 
pci_nvme_submission_queue_init(
        struct nvme_submission_queue_info* qinfo,
        struct blockif_ctxt* ctxt)
{
    struct nvme_ioreq *req;
    int ioq_size = blockif_queuesz(ctxt);
    qinfo->ioreq = calloc(ioq_size, sizeof(struct nvme_ioreq));
    STAILQ_INIT(&qinfo->iofhd);

    // setup and insert requests to the free queue
    for(int i=0;i<ioq_size; ++i)
    {
        req = &qinfo->ioreq[i] ;
        req->sq_info = qinfo;
        // setup callback function
        STAILQ_INSERT_TAIL(&qinfo->iofhd, req, io_flist);
    }

    TAILQ_INIT(&qinfo->iobhd);
    return 0;
}

static int 
pci_nvme_init (struct vmctx *ctx, struct pci_devinst *pi, char *opts)
{
    struct pci_nvme_softc *sc;
    struct blockif_ctxt *bctxt;

#ifdef NVME_DEBUG
    dbg = fopen("/tmp/nvme_emu_log", "w+");
    DPRINTF("--- start nvme controller ---\n");
#endif

    if(opts == NULL) {
        fprintf(stderr, "pci_nvme: backing device required\n");
        return 1;
    }
    DPRINTF("%s\n", opts);

    // XXX: what is a second argument of blockif_open?.
    bctxt = blockif_open(opts, ""); 
    if(bctxt == NULL)
    {
        goto fail;
    }

    pci_set_cfgdata16(pi, PCIR_DEVICE, 0x0111);
    pci_set_cfgdata16(pi, PCIR_VENDOR, 0x8086);
    // for NVMe Controller Registers
    if(pci_emul_alloc_bar(pi, 0, PCIBAR_MEM64, 0x100f))
    {
        DPRINTF("error is occured in pci_emul_alloc_bar\n");
        goto fail;
    }
    if(pci_emul_add_msixcap(pi, 4, 4)) //XXX fix msix num 
    {
        DPRINTF("error is occured in pci_emul_add_msixcap\n");
        goto fail;
    }
    DPRINTF("table %d, pba %d\n", pci_msix_table_bar(pi), pci_msix_pba_bar(pi));

    sc = calloc(1, sizeof(struct pci_nvme_softc));
    pi->pi_arg = sc;
    pi->pi_vmctx = ctx;
    sc->pi = pi;

    sc->bctx = bctxt;

    sc->regs.cap_hi.raw = 0x00000000;
    sc->regs.cap_lo.raw = 0x02000000;

    pci_nvme_reset(sc);

    int number_of_completion_queues = 4;
    sc->cqs_info = calloc(number_of_completion_queues, sizeof(struct nvme_completion_queue_info));

    int number_of_submission_queues = 4;
    sc->sqs_info = calloc(number_of_submission_queues, sizeof(struct nvme_submission_queue_info));

    for(int i=0; i<number_of_submission_queues; ++i)
    {
        pci_nvme_submission_queue_init(&sc->sqs_info[i], sc->bctx);
    }

    initialize_identify(sc);
    initialize_feature(sc);

    return 0;

fail:
    blockif_close(bctxt);
    free(sc->cqs_info);
    free(sc->sqs_info);
    free(sc);
    return 1;
}

static void
pci_nvme_setup_controller(struct vmctx *ctx, struct pci_nvme_softc *sc)
{
    DPRINTF("asqs 0x%x, acqs 0x%x\n", sc->regs.aqa.bits.asqs, sc->regs.aqa.bits.acqs);
    sc->asq_base = (uintptr_t)vm_map_gpa(
            ctx, sc->regs.asq, sizeof(struct nvme_command) * sc->regs.aqa.bits.asqs);
    sc->acq_base = (uintptr_t)vm_map_gpa(
            ctx, sc->regs.acq, sizeof(struct nvme_completion) * sc->regs.aqa.bits.acqs);

    sc->regs.csts.bits.rdy = 1;
}

static void 
execute_set_feature_command(struct pci_nvme_softc *sc,
        struct nvme_command *command, struct nvme_completion *cmp_entry)
{
    DPRINTF("0x%x\n", command->cdw11);
    DPRINTF("0x%x\n", command->cdw10);
    cmp_entry->cdw0 = 0x00000000;
    enum nvme_feature feature = command->cdw10 & 0xf;
    switch (feature) {
        case NVME_FEAT_NUMBER_OF_QUEUES:
            sc->features.num_of_queues.raw = command->cdw11 & 0xffff;
            DPRINTF("SET_FEATURE cmd: ncqr 0x%x, nsqr 0x%x\n", (command->cdw11 >> 16), (command->cdw11 & 0xffff));
            cmp_entry->status.sc = 0x00;
            cmp_entry->status.sct = 0x0;
            if(pci_msix_enabled(sc->pi)) {
                DPRINTF("generate msix, table_count %d, \n", sc->pi->pi_msix.table_count);
                pci_generate_msix(sc->pi, 0);
            }
            else {
                assert(0 && "pci_msix is disable?");
            }
            break;

        case NVME_FEAT_ASYNC_EVENT_CONFIGURATION:
            sc->features.async_event_config.raw = command->cdw11;
            cmp_entry->status.sc = 0x00;
            cmp_entry->status.sct = 0x0;
            pci_generate_msix(sc->pi, 0);
            break;

        case NVME_FEAT_INTERRUPT_COALESCING:
            DPRINTF("interrupt coalescing cdw11 0x%x\n", command->cdw11);
            cmp_entry->status.sc = 0x00;
            cmp_entry->status.sct = 0x0;
            sc->features.interrupt_coalscing.bits.thr = command->cdw11 & 0xff;
            sc->features.interrupt_coalscing.bits.time = (command->cdw11 >> 8) & 0xff;
            pci_generate_msix(sc->pi, 0);
            break;

        default:
            assert(0 && "this feature is not implemented");
    }
}

enum temp_threshold_cdw11 {
    NVME_TEMP_THRESHOLD_TMPTH   = 0x0000ffff,
    NVME_TEMP_THRESHOLD_TMPSEL  = 0x000f0000,
    NVME_TEMP_THRESHOLD_THSEL   = 0x00300000,
    NVME_TEMP_THRESHOLD_RESERVED= 0xffc00000,
};

static void 
execute_get_feature_command(struct pci_nvme_softc *sc,
        struct nvme_command *command, struct nvme_completion *cmp_entry)
{
    enum nvme_feature feature = command->cdw10 & 0xf;
    switch (feature)
    {
        case NVME_FEAT_TEMPERATURE_THRESHOLD:
            {
                uint8_t thsel = (command->cdw11 & NVME_TEMP_THRESHOLD_THSEL) >> 20;
                // over temparture threshold
                if(thsel == 0x00)
                {
                    cmp_entry->cdw0 = sc->features.temparture_threshold.bits.over;
                }
                // under temparture threshold
                else if(thsel == 0x1){
                    cmp_entry->cdw0 = sc->features.temparture_threshold.bits.under;
                }
                else {
                    assert("the thsel is invalied");
                }
                cmp_entry->status.sc = 0x00;
                cmp_entry->status.sct = 0x0;
                pci_generate_msix(sc->pi, 0);
                break;
            }

        default:
            DPRINTF("feature number: 0x%x\n", feature);
            assert(0 && "not implemented");
    }

    return;
}

static void
nvme_execute_identify_command(struct pci_nvme_softc *sc, 
        struct nvme_command *command, struct nvme_completion *cmp_entry)
{
    DPRINTF("Identify command (%s)\n", get_identify_command_type(command->cdw10 & NVME_CMD_IDENTIFY_CDW10_CNS));
    DPRINTF("cdw10 0x%x, dptr 0x%lx, 0x%lx", command->cdw10, command->prp1, command->prp2);
    uintptr_t dest_addr = (uintptr_t)vm_map_gpa(sc->pi->pi_vmctx,
            command->prp1, sizeof(struct nvme_controller_data));

    //TODO have to consider about completion queue entry content
    switch(command->cdw10 & NVME_CMD_IDENTIFY_CDW10_CNS)
    {
        case NVME_CMD_IDENTIFY_CNS_NAMESPACE:
            memcpy((struct nvme_namespace_data*)dest_addr,
                    &sc->namespace_data, sizeof(struct nvme_namespace_data));
            cmp_entry->status.sc = 0x00;
            cmp_entry->status.sct = 0x0;
            pci_generate_msix(sc->pi, 0);
            return;
        case NVME_CMD_IDENTIFY_CNS_CONTROLLER:
            memcpy((struct nvme_controller_data*)dest_addr,
                    &sc->controller_data, sizeof(struct nvme_controller_data));
            cmp_entry->status.sc = 0x00;
            cmp_entry->status.sct = 0x0;
            pci_generate_msix(sc->pi, 0);
            return;
        default:
            assert(0 && "[CNS] not inplemented");
    }

    assert(0);
}

enum create_io_cq_cdw11 {
    NVME_CREATE_IO_CQ_CDW11_PC  = 0x00000001, 
    NVME_CREATE_IO_CQ_CDW11_IEN = 0x00000002,
    NVME_CREATE_IO_CQ_CDW11_RSV = 0x0000fffc,
    NVME_CREATE_IO_CQ_CDW11_IV  = 0xffff0000,
};

static void 
nvme_execute_create_io_cq_command(struct pci_nvme_softc *sc, 
        struct nvme_command *command, struct nvme_completion *cmp_entry)
{
    //TODO
    //  IEN
    //  IV
    DPRINTF("interrupt vector 0x%x\n", command->cdw11 >> 16);
    if(command->cdw11 & NVME_CREATE_IO_CQ_CDW11_PC) {
        uint16_t qid = command->cdw10 & 0xffff;

        if(sc->cqs_info[qid].base_addr != (uintptr_t)NULL)
        {
            assert(0 && "the completion queue is already used");
        }

        uint16_t queue_size = command->cdw10 >> 16;
        sc->cqs_info[qid].base_addr = (uintptr_t)vm_map_gpa(
                sc->pi->pi_vmctx,
                command->prp1,
                sizeof(struct nvme_completion) * queue_size);
        sc->cqs_info[qid].size = queue_size;
        sc->cqs_info[qid].qid = qid;

        cmp_entry->status.sc = 0x00;
        cmp_entry->status.sct = 0x0;
        pci_generate_msix(sc->pi, 0);
    }
    else {
        assert(0 && "not implemented");
    }

    return;
}

enum create_io_sq_cdw11 {
    NVME_CREATE_IO_SQ_CDW11_PC      = 0x00000001,
    NVME_CREATE_IO_SQ_CDW11_QPRIO   = 0x00000060,
    NVME_CREATE_IO_SQ_CDW11_RSV     = 0x0000ff80,
    NVME_CREATE_IO_SQ_CDW11_CQID    = 0xffff0000,
};

static void
nvme_execute_create_io_sq_command(struct pci_nvme_softc *sc,
        struct nvme_command *command, struct nvme_completion *cmp_entry)
{
    if(command->cdw11 & NVME_CREATE_IO_SQ_CDW11_PC) {

        uint16_t qid = command->cdw10 & 0xffff;
        DPRINTF("qid: %x, base_addr %lx\n", qid, command->prp1);

        //TODO
/*         uint8_t queue_priority = (command->cdw11 & NVME_CREATE_IO_SQ_CDW11_QPRIO) >> 1; */
        struct nvme_submission_queue_info *sq_info = &sc->sqs_info[qid];
        if(sq_info->base_addr != (uintptr_t)NULL) 
        {
            assert(0);
        }
        uint16_t cqid = command->cdw11 >> 16;
        uint16_t queue_size = command->cdw10 >> 16;
        sq_info->base_addr = (uintptr_t)vm_map_gpa( 
                sc->pi->pi_vmctx,
                command->prp1,
                sizeof(struct nvme_command) * queue_size);
        sq_info->size = queue_size;
        sq_info->completion_qid = cqid;
        sq_info->qid = qid;

        DPRINTF("sc->sqs_info[%d].base_addr: %lx\n", qid, sc->sqs_info[qid].base_addr);

        cmp_entry->status.sc = 0x00;
        cmp_entry->status.sct = 0x0;
        pci_generate_msix(sc->pi, 0);
    }
    else {
        assert(0 && "not implemented");
    }
}

static void 
execute_async_event_request_command(struct pci_nvme_softc *sc,
        struct nvme_command *command, struct nvme_completion *cmp_entry)
{
    //TODO
    // when some events are occured, the controller send notify 
}

static void
pci_nvme_execute_admin_command(struct pci_nvme_softc * sc, uint64_t value)
{
    struct nvme_command *command = 
        (struct nvme_command *)(
                sc->asq_base + sizeof(struct nvme_command) * 
                (value - 1));

    struct nvme_completion *cmp_entry = 
        (struct nvme_completion *)(sc->acq_base + sizeof(struct nvme_completion) * sc->completion_queue_head);

    cmp_entry->sqid = 0;
    cmp_entry->sqhd = value - 1;
    cmp_entry->cid = command->cid;
    cmp_entry->status.p = !cmp_entry->status.p;

    DPRINTF("[admin command] %s\n", get_admin_command_text(command->opc));
    switch (command->opc)
    {
        case NVME_OPC_CREATE_IO_SQ:
            nvme_execute_create_io_sq_command(sc, command, cmp_entry);
            break;
        case NVME_OPC_CREATE_IO_CQ:
            nvme_execute_create_io_cq_command(sc, command, cmp_entry);
            break;
        case NVME_OPC_IDENTIFY:
            nvme_execute_identify_command(sc, command, cmp_entry);
            break;
        case NVME_OPC_SET_FEATURES:
            execute_set_feature_command(sc, command, cmp_entry);
            break;
        case NVME_OPC_GET_FEATURES:
            execute_get_feature_command(sc, command, cmp_entry);
            break;
        case NVME_OPC_ASYNC_EVENT_REQUEST:
            //XXX this is a dirty hack.... should be fixed...
            cmp_entry->status.p = !cmp_entry->status.p;
            execute_async_event_request_command(sc, command, cmp_entry);
            sc->completion_queue_head--;
            break;
        default:
            assert(0 && "the admin command is not implemented");
    }

    sc->completion_queue_head++;
}

static void
pci_nvme_blockif_ioreq_cb(struct blockif_req *br, int err)
{
    DPRINTF("%s %d\n", __func__, err);

    struct nvme_ioreq *nreq = br->br_param;
    struct nvme_completion_queue_info* cq_info = nreq->cq_info;
    struct nvme_submission_queue_info* sq_info = nreq->sq_info;

    pthread_mutex_lock(&cq_info->mtx);
    struct nvme_completion *completion_entry = 
        (struct nvme_completion *) (cq_info->base_addr + sizeof(struct nvme_completion) * cq_info->head);

    nreq->completion_entry.status.sct = 0x0;
    nreq->completion_entry.status.sc = 0x00;

    // save phase value before write values to completion queue entry
    uint8_t status_phase = completion_entry->status.p;

    memcpy(completion_entry, &nreq->completion_entry, sizeof(struct nvme_completion));
    completion_entry->status.p = !status_phase;

    cq_info->head++;
    if(cq_info->head > cq_info->size)
    {
        cq_info->head = 0;
    }

    TAILQ_REMOVE(&sq_info->iobhd, nreq, io_blist);

    STAILQ_INSERT_TAIL(&sq_info->iofhd, nreq, io_flist);

    pthread_mutex_unlock(&cq_info->mtx);
    pci_generate_msix(nreq->sc->pi, cq_info->qid);
}

static void
nvme_nvm_command_read_write(
        struct pci_nvme_softc *sc, 
        struct nvme_command *command, 
        struct nvme_submission_queue_info* sq_info,
        uint16_t sqhd)
{
    int err = 0;
    uintptr_t starting_lba = ((uint64_t)command->cdw11 << 32) | command->cdw10;
    // NLB (number of logic block) is a 0's based value
    uint16_t number_of_lb = (command->cdw12 & 0xffff) + 1;
    ssize_t logic_block_size = 1 << sc->namespace_data.lbaf[0].lbads;

    DPRINTF("slba %lx, nlba %x, size 2^%d\n", 
            starting_lba, number_of_lb, 
            sc->namespace_data.lbaf[0].lbads);
    DPRINTF("sqhd: 0x%x, destination addr : 0x%lx\n", sqhd, command->prp1);

/*     struct nvme_ioreq *nreq = sq_info->ioreq; */
    struct nvme_ioreq *nreq = STAILQ_FIRST(&sq_info->iofhd);
    assert(nreq != NULL);
    STAILQ_REMOVE_HEAD(&sq_info->iofhd, io_flist);

    nreq->completion_entry.sqhd = sqhd;
    nreq->completion_entry.sqid = sq_info->qid;
    nreq->completion_entry.cid = command->cid;
    nreq->cq_info = &sc->cqs_info[sq_info->completion_qid];
    nreq->sc = sc;

    struct blockif_req *breq = &nreq->io_req;
    breq->br_iovcnt = 1;
    breq->br_iov[0].iov_base = 
        paddr_guest2host(sc->pi->pi_vmctx, command->prp1, 
        (uint8_t)number_of_lb * logic_block_size);
    breq->br_iov[0].iov_len = number_of_lb * logic_block_size;
    breq->br_offset = starting_lba * logic_block_size;
    breq->br_resid = number_of_lb * logic_block_size;
    breq->br_callback = pci_nvme_blockif_ioreq_cb;
    breq->br_param = nreq;

    TAILQ_INSERT_HEAD(&sq_info->iobhd, nreq, io_blist);

    switch (command->opc)
    {
        case NVME_OPC_READ:
            err = blockif_read(sc->bctx, breq);
            break;
        case NVME_OPC_WRITE:
            err = blockif_write(sc->bctx, breq);
            break;
        default:
            assert("??");
    }

    assert(err == 0 && "blockif_read or blockif_write failed");
}

static void
nvme_nvm_command_flush(
        struct pci_nvme_softc *sc,
        struct nvme_submission_queue_info* sq_info,
        uint16_t cid,
        uint16_t sqhd)
{
    assert("not yet implemented");
    struct nvme_completion_queue_info *cq_info = &sc->cqs_info[sq_info->completion_qid];

    pthread_mutex_lock(&cq_info->mtx);
    struct nvme_completion *completion_entry = 
        (struct nvme_completion *)
        (cq_info->base_addr + sizeof(struct nvme_completion) * cq_info->head);

    completion_entry->sqhd = sqhd;
    completion_entry->sqid = sq_info->qid;
    completion_entry->cid = cid;
    completion_entry->status.sct = 0x0;
    completion_entry->status.sc = 0x00;
    completion_entry->status.p = !completion_entry->status.p;

    DPRINTF("cid: 0x%x, cqid: 0x%x, cq_info->head 0x%x, cq_info->size 0x%x\n", 
            cid, sq_info->completion_qid, cq_info->head, cq_info->size);
    cq_info->head++;
    if(cq_info->head == cq_info->size)
    {
        cq_info->head = 0;
    }
    pthread_mutex_unlock(&cq_info->mtx);

    pci_generate_msix(sc->pi, sq_info->completion_qid);
}

static void
pci_nvme_execute_nvme_command(struct pci_nvme_softc * sc, 
        uint16_t qid, uint64_t sq_tail)
{
    struct nvme_submission_queue_info *sq_info = &sc->sqs_info[qid];

    uint16_t command_index = sq_tail - 1;
    uint16_t sqhd = sq_tail - 1;
    if(sq_tail == 0x0)
    {
        command_index = sq_info->size; 
        sqhd = sq_info->size;
    }

    struct nvme_command* command = 
        (struct nvme_command*)(sq_info->base_addr + 
                sizeof(struct nvme_command) * (command_index));

/*     uint16_t completion_qid = sq_info->completion_qid; */
/*     struct nvme_completion_queue_info *cq_info = &sc->cqs_info[completion_qid]; */

    DPRINTF("***** nvm command %s *****\n", get_nvm_command_text(command->opc));
    DPRINTF("opc: 0x%x, cid: 0x%x, nsid: 0x%x, qid: 0x%x, value 0x%lx\n", 
            command->opc, command->cid, command->nsid, qid, sq_tail);

    switch (command->opc)
    {
        case NVME_OPC_READ:
        case NVME_OPC_WRITE:
            nvme_nvm_command_read_write(sc, command, sq_info, sqhd);
            return;
/*         case NVME_OPC_FLUSH: */
/*             nvme_nvm_command_flush(sc, sq_info, command->cid, sqhd); */
/*             return; */
/*         case NVME_OPC_DATASET_MANAGEMENT: */
/*             { */
/*                 pthread_mutex_lock(&cq_info->mtx); */
/*                 struct nvme_completion* completion_entry =  */
/*                     pci_nvme_acquire_completion_entry(sc, cq_info); */
/*                 completion_entry->status.sc = 0x00; */
/*                 completion_entry->status.sct = 0x0; */
/*                 pthread_mutex_unlock(&cq_info->mtx); */

/*                 pci_generate_msix(sc->pi, sq_info->completion_qid); */
/*                 return; */
/*             } */

        default:
            assert(0 && "the nvme command is not implemented yet");
    }
}

static void 
pci_nvme_write_bar_0(struct vmctx* ctx, struct pci_nvme_softc *sc, uint64_t regoff, uint64_t value, int size)
{

    // write value to doorbell register
    // TODO limit of doorbell register address is depend on the number of queue pairs.
    if(regoff >= NVME_CR_IO_QUEUE_BASE && regoff <= 0x100f)
    {
        int queue_offset = regoff - 0x1000; // 0x1000 is NVME_CR_IO_QUEUE_BASE
        DPRINTF("regoff 0x%lx\n", regoff);
        int qid = queue_offset / 8; // 4 is doorbell register size
        int is_completion = (queue_offset / 4) % 2;
        // completion doorbell
        if(is_completion)
        {
            DPRINTF("completion doorbell (%d) is knocked\n", qid);
            if(qid == 0)
            {
                //TODO
            }
            else {
                //TODO
            }
            return;
        }
        // submission doorbell
        else
        {
            // admin command
            if(qid == 0) 
            {
                pci_nvme_execute_admin_command(sc, value);
                return;
            }
            // nvme command
            // TODO validate qid
            else 
            {
                pci_nvme_execute_nvme_command(sc, qid, value);
                return;
            }
        }
        assert(0);
    }

    DPRINTF("write %s, value %lx \n", get_nvme_cr_text(regoff), value);
    assert(size == 4 && "word size should be 4(byte)");
    switch(regoff) 
    {
        case NVME_CR_CC:
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

            if((sc->regs.cc.bits.en == 0) && (value & NVME_CC_EN)) 
            {
                pci_nvme_reset(sc);
                pci_nvme_setup_controller(ctx, sc);
                DPRINTF("nvme up\n");
            }

            if((sc->regs.cc.bits.en == 1) && !(value & NVME_CC_EN))
            {
                DPRINTF("nvme down\n");
                sc->regs.csts.bits.rdy = 0;
                //TODO
                /*                     assert(0); */
            }

            sc->regs.cc.raw = (uint32_t)value;
            return;

        case NVME_CR_AQA:
            sc->regs.aqa.raw = (uint32_t)value;
            return;

        case NVME_CR_ASQ_LOW:
            sc->regs.asq = (sc->regs.asq & 0xffffffff00000000) | (0xfffff000 & value);
            return;

        case NVME_CR_ASQ_HI:
            sc->regs.asq = (sc->regs.asq & 0x00000000ffffffff) | (value << 32);
            return;

        case NVME_CR_ACQ_LOW:
            sc->regs.acq = (sc->regs.acq & 0xffffffff00000000) | (0xfffff000 &value);
            return;

        case NVME_CR_ACQ_HI:
            sc->regs.acq = (sc->regs.acq & 0x00000000ffffffff) | (value << 32);
            return;

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
        DPRINTF("baridx, %d, msix: regoff 0x%lx, size %d, value %lx\n", baridx, regoff, size, value);
        pci_emul_msix_twrite(pi, regoff, size, value);
        return;
    }

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
pci_nvme_read_bar_0(struct pci_nvme_softc *sc, enum nvme_controller_register_offsets offset, int size)
{
    DPRINTF("read %s\n", get_nvme_cr_text(offset));
    assert(size == 4 && "word size should be 4.");
    switch (offset) {
        case NVME_CR_CAP_LOW:
            return (uint64_t)sc->regs.cap_lo.raw;

        case NVME_CR_CAP_HI:
            return (uint64_t)sc->regs.cap_hi.raw;

        case NVME_CR_CC:
            return (uint64_t)sc->regs.cc.raw;

        case NVME_CR_CSTS:
            return (uint64_t)sc->regs.csts.raw;

        default:
            DPRINTF("unknown regoff value: 0x%x, size %d in %s\n", offset, size, __func__);
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
        DPRINTF("baridx: %d, msix: regoff 0x%lx, size %d\n", baridx, regoff, size);
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
