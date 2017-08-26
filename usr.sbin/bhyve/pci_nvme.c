#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <pthread.h>

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

enum nvme_doorbell_registers {
    NVME_DOORBELL_ADMIN_SUBMISSION = 0x1000,
    NVME_DOORBELL_ADMIN_COMPLETION = 0x1004,
    NVME_DOORBELL_NVM_SUBMISSION = 0x1008,
    NVME_DOORBELL_NVM_COMPLETION = 0x100c,
};

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

enum create_io_cq_cdw11 {
    NVME_CREATE_IO_CQ_CDW11_PC = 0x00000001,
    NVME_CREATE_IO_CQ_CDW11_IEN = 0x00000002,
    NVME_CREATE_IO_CQ_CDW11_RSV = 0x0000fffc,
    NVME_CREATE_IO_CQ_CDW11_IV = 0xffff0000,
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

const char* get_admin_command_text(enum nvme_admin_opcode opc)
{
    switch (opc) {
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
            assert(0 && "unknown opc\n");
    }
}

const char* get_feature_text(enum nvme_feature feature)
{
    switch (feature) {
        case NVME_FEAT_ARBITRATION:
            return "arbitation";
        case NVME_FEAT_POWER_MANAGEMENT:		
            return "power management";
        case NVME_FEAT_LBA_RANGE_TYPE:	
            return "LBA range type";
        case NVME_FEAT_TEMPERATURE_THRESHOLD:
            return "temperature threshold";
        case NVME_FEAT_ERROR_RECOVERY:	
            return "error recovery";
        case NVME_FEAT_VOLATILE_WRITE_CACHE:
            return "volatile write cache";
        case NVME_FEAT_NUMBER_OF_QUEUES:		
            return "number of queues";
        case NVME_FEAT_INTERRUPT_COALESCING:
            return "interrupt coalescing";
        case NVME_FEAT_INTERRUPT_VECTOR_CONFIGURATION:
            return "interrupt vector configuration";
        case NVME_FEAT_WRITE_ATOMICITY:		
            return "write atmicity";
        case NVME_FEAT_ASYNC_EVENT_CONFIGURATION:
            return "asynchronous event configuration";
        case NVME_FEAT_SOFTWARE_PROGRESS_MARKER:
            return "software progress makers";
        default:
            assert(0 && "unknown feature");
    }
}

#endif

#define MAX_CQ_NUM 2
#define MAX_SQ_NUM 2
#define DOORBELL_LIMIT 0x100f

struct nvme_features {
    union {
        uint32_t raw;
        struct {
            uint16_t ncqr;
            uint16_t nsqr;
        } __packed bits;
    } __packed num_of_queues;

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
    uint16_t tail;
};

struct nvme_submission_queue {
    uintptr_t base_addr;
    uint16_t size;
    uint16_t qid;
    uint16_t completion_qid;
    uint16_t tail;
    pthread_cond_t cond;
    pthread_mutex_t mtx;
    //XX ioreq
};

struct pci_nvme_softc {
    struct pci_devinst* pi;
    struct blockif_ctxt* bctx;
    struct nvme_registers regs;
    struct nvme_completion_queue* cqs;
    struct nvme_submission_queue* sqs;
    struct nvme_controller_data controller_data;
    struct nvme_namespace_data namespace_data;
    struct nvme_features features;
    pthread_t sq_work_thread[MAX_SQ_NUM];
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

/* struct nvme_submission_queue { */
/*     uintptr_t base_addr; */
/*     uint16_t size; */
/*     uint16_t qid; */
/*     uint16_t completion_qid; */
/*     uint16_t tail; */
/*     pthread_cond_t cond; */
/*     pthread_mutex_t mtx; */
/*     //XX ioreq */
/* }; */


static void
pci_nvme_sq_init(uint16_t size, int qid, int cqid,
        struct nvme_submission_queue* queue_info)
{
    queue_info->base_addr = (uintptr_t)NULL;
    queue_info->size = size;
    queue_info->qid = qid;
    queue_info->completion_qid = cqid;
    queue_info->tail = 0;
    pthread_cond_init(&queue_info->cond, NULL);
    pthread_mutex_init(&queue_info->mtx, NULL);
    //initialize ioreq for blockif
}

static void
pci_nvme_cq_init(uint16_t size, int qid, int interrupt_vector, 
        struct nvme_completion_queue* cq_info)
{
    cq_info->base_addr = (uintptr_t)NULL;
    cq_info->size = size;
    cq_info->qid = qid;
    cq_info->tail = 0;
    cq_info->interrupt_vector = interrupt_vector;
}

enum create_io_sq_cdw11 {
    NVME_CREATE_IO_SQ_CDW11_PC = 0x00000001,
    NVME_CREATE_IO_SQ_CDW11_QPRIO = 0x00000060,
    NVME_CREATE_IO_SQ_CDW11_RSV = 0x0000ff80,
    NVME_CREATE_IO_SQ_CDW11_CQID = 0xffff0000,
};

static void 
pci_nvme_execute_create_sq_command(struct pci_nvme_softc* sc,
                                              struct nvme_command* command,
                                              struct nvme_completion* cmp_entry)
{
    if (command->cdw11 & NVME_CREATE_IO_SQ_CDW11_PC) {
        uint16_t qid = command->cdw10 & 0xffff;

        // TODO
        /*         uint8_t queue_priority = (command->cdw11 &
         * NVME_CREATE_IO_SQ_CDW11_QPRIO) >> 1; */
        struct nvme_submission_queue* sq_info = &sc->sqs[qid];
        if (sq_info->base_addr != (uintptr_t)NULL) {
            assert(0);
        }
        uint16_t cqid = command->cdw11 >> 16;
        uint16_t queue_size = command->cdw10 >> 16;
        sq_info->base_addr =
            (uintptr_t)paddr_guest2host(sc->pi->pi_vmctx, command->prp1,
                                  sizeof(struct nvme_command) * queue_size);
        sq_info->size = queue_size;
        sq_info->completion_qid = cqid;
        sq_info->qid = qid;

        cmp_entry->status.sc = 0x00;
        cmp_entry->status.sct = 0x0;
    }
    else {
        assert(0 && "not implemented");
    }

}

static void
pci_nvme_execute_set_feature_command(struct pci_nvme_softc* sc,
        struct nvme_command *command,
        struct nvme_completion* cmp_entry)
{
    cmp_entry->cdw0 = 0x00000000;
    enum nvme_feature feature = command->cdw10 & 0xf;
    DPRINTF("set feature [%s]\n", get_feature_text(feature));
    switch (feature) {
        case NVME_FEAT_NUMBER_OF_QUEUES:
            sc->features.num_of_queues.raw = command->cdw11;
            DPRINTF("number of io queues: CQ 0x%x, SQ 0x%x\n",
                    sc->features.num_of_queues.bits.ncqr, 
                    sc->features.num_of_queues.bits.nsqr);
            cmp_entry->status.sc = 0x00;
            cmp_entry->status.sct = 0x0;
/*             cmp_entry->cdw0 = (0x0001 << 16) || 0x0001; */
            cmp_entry->cdw0 = 0;
            break;

/*         case NVME_FEAT_ASYNC_EVENT_CONFIGURATION: */
/*             //TODO  */
/*             sc->features.async_event_config.raw = command->cdw11; */
/*             cmp_entry->status.sc = 0x00; */
/*             cmp_entry->status.sct = 0x0; */
/*             pci_generate_msix(sc->pi, 0); */
/*             break; */

/*         case NVME_FEAT_INTERRUPT_COALESCING: */
/*             DPRINTF("interrupt coalescing cdw11 0x%x\n", command->cdw11); */
/*             cmp_entry->status.sc = 0x00; */
/*             cmp_entry->status.sct = 0x0; */
/*             sc->features.interrupt_coalscing.bits.thr = command->cdw11 & 0xff; */
/*             sc->features.interrupt_coalscing.bits.time = */
/*                 (command->cdw11 >> 8) & 0xff; */
/*             pci_generate_msix(sc->pi, 0); */
/*             break; */

        default:
            assert(0 && "this feature is not implemented");
    }
}

enum nvme_cmd_identify_data {
    NVME_CMD_IDENTIFY_CNS_NAMESPACE = 0x0,
    NVME_CMD_IDENTIFY_CNS_CONTROLLER = 0x1,
    NVME_CMD_IDENTIFY_CNS_NAMESPACE_LIST = 0x2,
    NVME_CMD_IDENTIFY_CNS_RESERVED = 0x3,
};

enum nvme_cmd_identify_cdw10 {
    NVME_CMD_IDENTIFY_CDW10_CNTID = 0xffff0000,
    NVME_CMD_IDENTIFY_CDW10_RSV = 0x0000ff00,
    NVME_CMD_IDENTIFY_CDW10_CNS = 0x000000ff,
};

static void
pci_nvme_execute_identify_command(struct pci_nvme_softc* sc,
        struct nvme_command *command, 
        struct nvme_completion* cmp_entry)
{
    uintptr_t dest_addr = (uintptr_t)paddr_guest2host(
        ctx_from_sc(sc), command->prp1, sizeof(struct nvme_controller_data));

    enum nvme_cmd_identify_data identify =
        command->cdw10 & NVME_CMD_IDENTIFY_CDW10_CNS ;

    switch(identify) {
        case NVME_CMD_IDENTIFY_CNS_NAMESPACE:
            memcpy((struct nvme_namespace_data*)dest_addr, &sc->namespace_data,
                   sizeof(struct nvme_namespace_data));
            cmp_entry->status.sc = 0x00;
            cmp_entry->status.sct = 0x0;
            break;
        case NVME_CMD_IDENTIFY_CNS_CONTROLLER:
            memcpy((struct nvme_controller_data*)dest_addr,
                   &sc->controller_data, sizeof(struct nvme_controller_data));
            cmp_entry->status.sc = 0x00;
            cmp_entry->status.sct = 0x0;
            break;
        default:
            assert(0);
    }
}

static void
pci_nvme_execute_create_cq_command(struct pci_nvme_softc* sc,
        struct nvme_command *command,
        struct nvme_completion* cmp_entry)
{
    if (command->cdw11 & NVME_CREATE_IO_CQ_CDW11_PC) {
        uint16_t qid = command->cdw10 & 0xffff;

        if (sc->cqs[qid].base_addr != (uintptr_t)NULL) {
            assert(0 && "the completion queue is already used");
        }

        uint16_t interrupt_vector = command->cdw11 >> 16;
        uint16_t queue_size = command->cdw10 >> 16;
        sc->cqs[qid].base_addr =
            (uintptr_t)paddr_guest2host(ctx_from_sc(sc), command->prp1,
                                  sizeof(struct nvme_completion) * queue_size);
        sc->cqs[qid].size = queue_size;
        sc->cqs[qid].qid = qid;
        sc->cqs[qid].interrupt_vector = interrupt_vector;

        cmp_entry->status.sc = 0x00;
        cmp_entry->status.sct = 0x0;
    }
    else {
        assert(0 && "not implemented");
    }
}

static void
pci_nvme_admin_cmd_execute(struct pci_nvme_softc* sc, uint16_t* sq_head)
{
    struct nvme_command* command =
        (struct nvme_command*)(sc->sqs[0].base_addr +
                               sizeof(struct nvme_command) * *sq_head);
    struct nvme_completion_queue* admin_cq = &sc->cqs[0];
    struct nvme_completion* completion_entry =
        (struct nvme_completion*)(sc->cqs[0].base_addr +
                                  sizeof(struct nvme_completion) *
                                      admin_cq->tail);

    DPRINTF("admin cmd exec [%s]\n", get_admin_command_text(command->opc));
    switch (command->opc) {
        case NVME_OPC_CREATE_IO_SQ:
            pci_nvme_execute_create_sq_command(sc, command,
                                                  completion_entry);
            break;
        case NVME_OPC_CREATE_IO_CQ:
            pci_nvme_execute_create_cq_command(sc, command, completion_entry);
            break;
        case NVME_OPC_IDENTIFY:
            pci_nvme_execute_identify_command(sc, command, completion_entry);
            break;
        case NVME_OPC_SET_FEATURES:
            pci_nvme_execute_set_feature_command(sc, command, completion_entry);
            break;

        default:
            assert(0 && "the admin command is not yet implemented");
    }

    if(command->opc != NVME_OPC_ASYNC_EVENT_REQUEST) {
        completion_entry->sqid = 0;
        completion_entry->sqhd = *sq_head;
        completion_entry->cid = command->cid;
        completion_entry->status.p = !completion_entry->status.p;

        admin_cq->tail++;
        if(sc->regs.aqa.bits.acqs == admin_cq->tail)
        {
            admin_cq->tail = 0;
        }

        pci_generate_msix(sc->pi, 0);
    }

    (*sq_head)++;
    if(*sq_head == sc->regs.aqa.bits.asqs) 
    {
        *sq_head = 0;
    }
}

static void *
pci_nvme_admin_cmd_exec_thr(void* arg)
{
    struct pci_nvme_softc* sc = arg;
    uint16_t sq_head = 0;
    struct nvme_submission_queue* admin_sq = &sc->sqs[0];

    pthread_mutex_lock(&admin_sq->mtx);
    while(true) {
        while(sq_head == admin_sq->tail) {
            pthread_cond_wait(&admin_sq->cond, &admin_sq->mtx);
        }
        pci_nvme_admin_cmd_execute(sc, &sq_head);
        DPRINTF("cq tail is %x\n", sc->cqs[0].tail);
    }
    pthread_mutex_unlock(&admin_sq->mtx);
}

static int 
pci_nvme_init(struct vmctx* ctx, struct pci_devinst* pi, char* opts)
{
    struct pci_nvme_softc* sc;
    struct blockif_ctxt* bctx;
    int status;

#ifdef NVME_DEBUG
    dbg = fopen("/tmp/nvme_emul_log", "w+");
#endif
    DPRINTF("----- initialize nvme controller emulator -----\n");

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
     */

    status = pci_emul_alloc_bar(pi, 0, PCIBAR_MEM64, 0x100f);
/*     status = pci_emul_alloc_bar(pi, 0, PCIBAR_MEM32, DOORBELL_LIMIT); */
    if (status) {
        DPRINTF("Error is occurred in pci_emul_alloc_bar\n");
        return 1;
    }

    status = pci_emul_add_msixcap(pi, 4, 4);
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

    /* setup admin queues */
    pci_nvme_sq_init(sc->regs.aqa.bits.asqs, 0, 0, &sc->sqs[0]);
    pci_nvme_cq_init(sc->regs.aqa.bits.acqs, 0, 0, &sc->cqs[0]);
    pthread_create(sc->sq_work_thread, NULL,
            pci_nvme_admin_cmd_exec_thr, sc);
   
    for (int i = 1; i < MAX_SQ_NUM; ++i) {
        // TODO: Initialize submittion i/o queue
    }
    for (int i = 1; i < MAX_CQ_NUM; ++i) {
        // TODO: Initialize completion i/o queue
    }

    return 0;

free_sc:
    free(sc);
    return 1;
}

static void
pci_nvme_setup_controller(struct pci_nvme_softc* sc)
{
    sc->sqs[0].base_addr =
        (uintptr_t)paddr_guest2host(ctx_from_sc(sc), sc->regs.asq,
                sizeof(struct nvme_command) * sc->regs.aqa.bits.asqs);
    sc->cqs[0].base_addr = 
        (uintptr_t)paddr_guest2host(ctx_from_sc(sc), sc->regs.acq,
                sizeof(struct nvme_completion) * sc->regs.aqa.bits.acqs);

    DPRINTF("asq 0x%lx, acq 0x%lx in guest\n", sc->regs.asq, sc->regs.acq);
    DPRINTF("admin sq base_addr 0x%lx, cq base_addr 0x%lx in host\n",
            sc->sqs[0].base_addr, sc->cqs[0].base_addr);

    sc->regs.csts.bits.rdy = 1;
}

static void
pci_nvme_write_bar_0(struct vmctx* ctx, struct pci_nvme_softc* sc,
        uint64_t offset, uint64_t value, int size)
{

    struct nvme_submission_queue* admin_sq = &sc->sqs[0];
    /*
     * Guest write to doorbell registers
     * - offset
     *  0x1000 ~ DOORBELL_LIMIT(0x100f)
     */
    DPRINTF("%s, value is %lx\n", __func__, value);
    if (offset >= NVME_CR_IO_QUEUE_BASE && offset <= DOORBELL_LIMIT) {
        switch(offset) {
            case NVME_DOORBELL_ADMIN_SUBMISSION:
                admin_sq->tail = value;
                pthread_cond_signal(&admin_sq->cond);
                return;
            case NVME_DOORBELL_ADMIN_COMPLETION:
                DPRINTF("Doorbell is knocked for admin completion.\n");
                DPRINTF("CQ head is 0x%lx\n", value);
                return;
            case NVME_DOORBELL_NVM_SUBMISSION:
            case NVME_DOORBELL_NVM_COMPLETION:

            default:
                assert(0);
        }
    }

    /*
     * Guest access to NVMe controller reigsters
     * - offset
     *   0x0000 ~ 0x1000
     */
    DPRINTF("write %s , value is %lx\n", 
            get_nvme_cr_text(offset, false), value);
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
                DPRINTF("nvme up\n");
/*                 pci_nvme_softc_reset(sc); */
                pci_nvme_setup_controller(sc);
            }
            if(sc->regs.cc.bits.en && !(value & NVME_CC_EN)) {
                DPRINTF("nvme down\n");
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

        /*
         * Registers for admin submission queue and completion queue.
         */
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

    if(baridx == pci_msix_table_bar(pi) || baridx == pci_msix_pba_bar(pi)) {
        DPRINTF("twrite baridx %d, offset 0x%lx, size %d, value %lx\n",
                baridx, offset, size, value);
        pci_emul_msix_twrite(pi, offset, size, value);
        return;
    }

    switch (baridx) {
        case 0:
            pci_nvme_write_bar_0(ctx, sc, offset, value, size);
            break;

        default:
            DPRINTF("unknown baridx: 0x%x, size %d in %s\n", baridx, size,
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
            DPRINTF("CC raw 0x%x\n", sc->regs.cc.raw);
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
        DPRINTF("tread baridx %d, offset 0x%lx, size %d\n",
                baridx, regoff, size);
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
