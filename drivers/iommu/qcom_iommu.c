/*
 * IOMMU driver for QCOM secure IOMMUs (MSM8916/MSM8909/APQ8009).
 *
 * Backported from mainline Linux (drivers/iommu/arm/arm-smmu/qcom_iommu.c)
 * to kernel 4.9 by adapting the IOMMU and io-pgtable APIs.
 *
 * Original:
 *   Copyright (C) 2013 ARM Limited
 *   Copyright (C) 2017 Red Hat
 * Backport:
 *   2026 — adapted for LA.UM.8.7 / kernel 4.9
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#define pr_fmt(fmt)	"qcom-iommu: " fmt

#include <linux/atomic.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/io-64-nonatomic-hi-lo.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_iommu.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <soc/qcom/scm.h>

#include "io-pgtable.h"

/* ------------------------------------------------------------------ */
/* Context-bank register offsets (from arm-smmu.c, no GR0 needed)     */
/* ------------------------------------------------------------------ */
#define ARM_SMMU_CB_SCTLR		0x0
#define ARM_SMMU_CB_RESUME		0x8
#define ARM_SMMU_CB_TTBCR2		0x10
#define ARM_SMMU_CB_TTBR0		0x20
#define ARM_SMMU_CB_TTBR1		0x28
#define ARM_SMMU_CB_TTBCR		0x30
#define ARM_SMMU_CB_S1_MAIR0		0x38
#define ARM_SMMU_CB_S1_MAIR1		0x3c
#define ARM_SMMU_CB_FSR			0x58
#define ARM_SMMU_CB_FAR			0x60
#define ARM_SMMU_CB_FSYNR0		0x68
#define ARM_SMMU_CB_S1_TLBIVA		0x600
#define ARM_SMMU_CB_S1_TLBIASID		0x610
#define ARM_SMMU_CB_S1_TLBIVAL		0x620
#define ARM_SMMU_CB_TLBSYNC		0x7f0
#define ARM_SMMU_CB_TLBSTATUS		0x7f4

#define SCTLR_S1_ASIDPNE		(1 << 12)
#define SCTLR_CFCFG			(1 << 7)
#define SCTLR_CFIE			(1 << 6)
#define SCTLR_CFRE			(1 << 5)
#define SCTLR_E			(1 << 4)
#define SCTLR_AFE			(1 << 2)
#define SCTLR_TRE			(1 << 1)
#define SCTLR_M			(1 << 0)

#define RESUME_TERMINATE		(1 << 0)
#define TTBRn_ASID_SHIFT		48

#define FSR_MULTI			(1 << 31)
#define FSR_SS				(1 << 30)
#define FSR_UUT				(1 << 8)
#define FSR_ASF				(1 << 7)
#define FSR_TLBLKF			(1 << 6)
#define FSR_TLBMCF			(1 << 5)
#define FSR_EF				(1 << 4)
#define FSR_PF				(1 << 3)
#define FSR_AFF				(1 << 2)
#define FSR_TF				(1 << 1)
#define FSR_FAULT			(FSR_MULTI | FSR_SS | FSR_UUT | \
					 FSR_ASF | FSR_TLBLKF | FSR_TLBMCF | \
					 FSR_EF | FSR_PF | FSR_AFF | FSR_TF)

#define SMMU_INTR_SEL_NS		0x2000

/* ------------------------------------------------------------------ */
/* Data structures                                                    */
/* ------------------------------------------------------------------ */
struct qcom_iommu_ctx;

struct qcom_iommu_dev {
	struct device		*dev;
	struct clk		*clk_iface;
	struct clk		*clk_bus;
	struct regulator	*vdd;	/* opt: GPU GDSC (oxili_gx) for kgsl_smmu */
	void __iomem		*local_base;
	u32			 sec_id;
	u8			 max_asid;
	struct qcom_iommu_ctx	*ctxs[];	/* indexed by asid, flex array */
};

struct qcom_iommu_ctx {
	struct device		*dev;
	void __iomem		*base;
	bool			 secure_init;
	bool			 secured_ctx;
	u8			 asid;
	u8			 cb;	/* HW context-bank index (reg / 0x1000) */
	struct iommu_domain	*domain;
};

struct qcom_iommu_domain {
	struct io_pgtable_ops	*pgtbl_ops;
	spinlock_t		 pgtbl_lock;
	struct mutex		 init_mutex;
	struct iommu_domain	 domain;
	struct qcom_iommu_dev	*iommu;
	struct iommu_fwspec	*fwspec;
	u64			 ttbr0;	/* exposed via DOMAIN_ATTR_TTBR0 (KGSL) */
	bool			 cb_stall_disable;
};

static struct iommu_ops qcom_iommu_ops;

static struct qcom_iommu_domain *to_qcom_iommu_domain(struct iommu_domain *dom)
{
	return container_of(dom, struct qcom_iommu_domain, domain);
}

static struct qcom_iommu_ctx *to_ctx(struct qcom_iommu_domain *d,
				     unsigned int asid)
{
	struct qcom_iommu_dev *qcom_iommu = d->iommu;

	if (!qcom_iommu)
		return NULL;
	return qcom_iommu->ctxs[asid];
}

/* ------------------------------------------------------------------ */
/* Register helpers — context-bank only, never GR0                    */
/* ------------------------------------------------------------------ */
static inline void ctx_writel(struct qcom_iommu_ctx *ctx, u32 reg, u32 val)
{
	writel_relaxed(val, ctx->base + reg);
}

static inline void ctx_writeq(struct qcom_iommu_ctx *ctx, u32 reg, u64 val)
{
	writeq_relaxed(val, ctx->base + reg);
}

static inline u32 ctx_readl(struct qcom_iommu_ctx *ctx, u32 reg)
{
	return readl_relaxed(ctx->base + reg);
}

/* ------------------------------------------------------------------ */
/* io-pgtable TLB callbacks (4.9 API: iommu_gather_ops)               */
/* ------------------------------------------------------------------ */
static void qcom_iommu_tlb_sync(void *cookie)
{
	struct qcom_iommu_domain *qcom_domain = cookie;
	struct iommu_fwspec *fwspec = qcom_domain->fwspec;
	unsigned int i;

	for (i = 0; i < fwspec->num_ids; i++) {
		struct qcom_iommu_ctx *ctx = to_ctx(qcom_domain,
						    fwspec->ids[i]);
		unsigned int val, ret;

		ctx_writel(ctx, ARM_SMMU_CB_TLBSYNC, 0);

		ret = readl_poll_timeout(ctx->base + ARM_SMMU_CB_TLBSTATUS,
					 val, (val & 0x1) == 0, 0, 5000000);
		if (ret)
			dev_err(ctx->dev, "timeout waiting for TLB SYNC\n");
	}
}

static void qcom_iommu_tlb_inv_context(void *cookie)
{
	struct qcom_iommu_domain *qcom_domain = cookie;
	struct iommu_fwspec *fwspec = qcom_domain->fwspec;
	unsigned int i;

	for (i = 0; i < fwspec->num_ids; i++) {
		struct qcom_iommu_ctx *ctx = to_ctx(qcom_domain,
						    fwspec->ids[i]);
		ctx_writel(ctx, ARM_SMMU_CB_S1_TLBIASID, ctx->asid);
	}

	qcom_iommu_tlb_sync(cookie);
}

static void qcom_iommu_tlb_inv_range_nosync(unsigned long iova, size_t size,
					    size_t granule, bool leaf,
					    void *cookie)
{
	struct qcom_iommu_domain *qcom_domain = cookie;
	struct iommu_fwspec *fwspec = qcom_domain->fwspec;
	unsigned int i, reg;

	reg = leaf ? ARM_SMMU_CB_S1_TLBIVAL : ARM_SMMU_CB_S1_TLBIVA;

	for (i = 0; i < fwspec->num_ids; i++) {
		struct qcom_iommu_ctx *ctx = to_ctx(qcom_domain,
						    fwspec->ids[i]);
		size_t s = size;

		iova = (iova >> 12) << 12;
		iova |= ctx->asid;
		do {
			ctx_writel(ctx, reg, iova);
			iova += granule;
		} while (s -= granule);
	}
}

static struct iommu_gather_ops qcom_gather_ops = {
	.tlb_flush_all	= qcom_iommu_tlb_inv_context,
	.tlb_add_flush	= qcom_iommu_tlb_inv_range_nosync,
	.tlb_sync	= qcom_iommu_tlb_sync,
};

/* ------------------------------------------------------------------ */
/* Context-fault IRQ                                                  */
/* ------------------------------------------------------------------ */
static irqreturn_t qcom_iommu_fault(int irq, void *dev)
{
	struct qcom_iommu_ctx *ctx = dev;
	u32 fsr, fsynr;
	u64 iova;

	fsr = ctx_readl(ctx, ARM_SMMU_CB_FSR);
	if (!(fsr & FSR_FAULT))
		return IRQ_NONE;

	fsynr = ctx_readl(ctx, ARM_SMMU_CB_FSYNR0);
	iova = readq_relaxed(ctx->base + ARM_SMMU_CB_FAR);

	if (!report_iommu_fault(ctx->domain, ctx->dev, iova, 0)) {
		dev_err_ratelimited(ctx->dev,
				    "Unhandled context fault: fsr=0x%x, "
				    "iova=0x%016llx, fsynr=0x%x, cb=%d\n",
				    fsr, iova, fsynr, ctx->asid);
	}

	ctx_writel(ctx, ARM_SMMU_CB_FSR, fsr);
	ctx_writel(ctx, ARM_SMMU_CB_RESUME, RESUME_TERMINATE);

	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------ */
/* Domain init — programs the context banks                           */
/* ------------------------------------------------------------------ */
static int qcom_iommu_init_domain(struct iommu_domain *domain,
				  struct qcom_iommu_dev *qcom_iommu,
				  struct device *dev)
{
	struct qcom_iommu_domain *qcom_domain = to_qcom_iommu_domain(domain);
	struct iommu_fwspec *fwspec = dev->iommu_fwspec;
	struct io_pgtable_ops *pgtbl_ops;
	struct io_pgtable_cfg pgtbl_cfg;
	int i, ret = 0;
	u32 reg;

	mutex_lock(&qcom_domain->init_mutex);
	if (qcom_domain->iommu) {
		/*
		 * RE-ATTACH: el dominio ya tiene page tables asignadas, pero un
		 * detach previo deshabilito el/los context bank(s) (SCTLR=0,
		 * ctx->domain=NULL) -- ver qcom_iommu_detach_dev(). El SMMU de
		 * apps es always-on (runtime-suspend solo gatea clocks, no
		 * power-collapse) => TTBR/TTBCR/MAIR del CB sobreviven; basta
		 * re-habilitar SCTLR y re-bindear el ctx para restaurar la
		 * traduccion. Sin esto, la 2a (y siguientes) sesion de un cliente
		 * que hace attach/detach repetidos (camara VFE) corre con el CB
		 * en BYPASS: las IOVA del buf_mgr (0x10000000..0x80000000, por
		 * debajo de la DRAM) pasan sin traducir, las escrituras DMA del
		 * VFE se pierden y los frames salen en cero (verdes). Solo un
		 * reboot reasignaba el dominio. (X104 reopen-green fix.)
		 */
		for (i = 0; i < fwspec->num_ids; i++) {
			struct qcom_iommu_ctx *ctx =
				to_ctx(qcom_domain, fwspec->ids[i]);

			reg = SCTLR_CFIE | SCTLR_CFRE | SCTLR_AFE |
			      SCTLR_TRE | SCTLR_M | SCTLR_S1_ASIDPNE |
			      SCTLR_CFCFG;
			if (IS_ENABLED(CONFIG_CPU_BIG_ENDIAN))
				reg |= SCTLR_E;
			if (qcom_domain->cb_stall_disable)
				reg &= ~SCTLR_CFCFG;
			ctx_writel(ctx, ARM_SMMU_CB_SCTLR, reg);
			ctx->domain = domain;
		}
		goto out_unlock;
	}

	pgtbl_cfg = (struct io_pgtable_cfg) {
		.pgsize_bitmap	= domain->pgsize_bitmap,
		.ias		= 32,
		.oas		= 40,
		.tlb		= &qcom_gather_ops,
		.iommu_dev	= qcom_iommu->dev,
	};

	qcom_domain->iommu = qcom_iommu;
	qcom_domain->fwspec = fwspec;

	pgtbl_ops = alloc_io_pgtable_ops(ARM_32_LPAE_S1, &pgtbl_cfg,
					 qcom_domain);
	if (!pgtbl_ops) {
		dev_err(qcom_iommu->dev,
			"failed to allocate pagetable ops\n");
		ret = -ENOMEM;
		goto out_clear_iommu;
	}

	domain->geometry.aperture_end = (1ULL << pgtbl_cfg.ias) - 1;
	domain->geometry.force_aperture = true;

	for (i = 0; i < fwspec->num_ids; i++) {
		struct qcom_iommu_ctx *ctx = to_ctx(qcom_domain,
						    fwspec->ids[i]);
		int scm_ret = 0;

		if (!ctx->secure_init) {
			ret = scm_restore_sec_cfg(qcom_iommu->sec_id,
						  ctx->asid, &scm_ret);
			if (ret || scm_ret) {
				dev_err(qcom_iommu->dev,
					"secure init failed: %d/%d\n",
					ret, scm_ret);
				ret = ret ? ret : -EINVAL;
				goto out_clear_iommu;
			}
			ctx->secure_init = true;
		}

		/* Secured contexts cannot be programmed by HLOS */
		if (ctx->secured_ctx) {
			ctx->domain = domain;
			continue;
		}

		/* Disable context bank before programming */
		ctx_writel(ctx, ARM_SMMU_CB_SCTLR, 0);

		/* Clear fault status */
		ctx_writel(ctx, ARM_SMMU_CB_FAR, 0);
		ctx_writel(ctx, ARM_SMMU_CB_FSR, FSR_FAULT);

		/* TTBCR2 (upper 32 bits of TCR) */
		ctx_writel(ctx, ARM_SMMU_CB_TTBCR2,
			   pgtbl_cfg.arm_lpae_s1_cfg.tcr >> 32);

		/* TTBCR (lower 32 bits of TCR) */
		ctx_writel(ctx, ARM_SMMU_CB_TTBCR,
			   pgtbl_cfg.arm_lpae_s1_cfg.tcr);

		/* TTBRs */
		ctx_writeq(ctx, ARM_SMMU_CB_TTBR0,
			   pgtbl_cfg.arm_lpae_s1_cfg.ttbr[0] |
			   ((u64)ctx->asid << TTBRn_ASID_SHIFT));
		ctx_writeq(ctx, ARM_SMMU_CB_TTBR1, 0);

		/* Expose TTBR0 to KGSL (global-pagetable mode) */
		qcom_domain->ttbr0 = pgtbl_cfg.arm_lpae_s1_cfg.ttbr[0] |
			   ((u64)ctx->asid << TTBRn_ASID_SHIFT);

		/* MAIRs (stage-1 only) */
		ctx_writel(ctx, ARM_SMMU_CB_S1_MAIR0,
			   pgtbl_cfg.arm_lpae_s1_cfg.mair[0]);
		ctx_writel(ctx, ARM_SMMU_CB_S1_MAIR1,
			   pgtbl_cfg.arm_lpae_s1_cfg.mair[1]);

		/* SCTLR */
		reg = SCTLR_CFIE | SCTLR_CFRE | SCTLR_AFE |
		      SCTLR_TRE | SCTLR_M | SCTLR_S1_ASIDPNE |
		      SCTLR_CFCFG;

		if (IS_ENABLED(CONFIG_CPU_BIG_ENDIAN))
			reg |= SCTLR_E;
		if (qcom_domain->cb_stall_disable)
			reg &= ~SCTLR_CFCFG;

		ctx_writel(ctx, ARM_SMMU_CB_SCTLR, reg);

		ctx->domain = domain;
	}

	mutex_unlock(&qcom_domain->init_mutex);

	/* Publish page table ops for map/unmap */
	qcom_domain->pgtbl_ops = pgtbl_ops;

	return 0;

out_clear_iommu:
	qcom_domain->iommu = NULL;
out_unlock:
	mutex_unlock(&qcom_domain->init_mutex);
	return ret;
}

/* ------------------------------------------------------------------ */
/* IOMMU domain ops                                                   */
/* ------------------------------------------------------------------ */
static struct iommu_domain *qcom_iommu_domain_alloc(unsigned int type)
{
	struct qcom_iommu_domain *qcom_domain;

	/*
	 * Only hand out UNMANAGED domains. Every consumer on this SMMU
	 * (kgsl/mdss/vidc/cam) drives its own UNMANAGED domain explicitly:
	 * KGSL via iommu_attach_device(), and mdss/vidc/cam via the ARM
	 * dma-mapping wrapper arm_iommu_attach_device(). Refusing
	 * IOMMU_DOMAIN_DMA here makes iommu_group_get_for_dev() leave
	 * group->default_domain == NULL, so the core never auto-attaches an
	 * (unprogrammed) default domain to the context device. That matters
	 * because arm_iommu_attach_device() hard-rejects a device that
	 * already reports a domain ("Device already attached to other
	 * iommu_domain", -EINVAL) -> mdp3 fell back to bypass and could not
	 * map overlay buffers (atomic commit -22 loop); vidc CBs failed the
	 * same way. The default DMA domain was never programmed anyway (see
	 * the IOMMU_DOMAIN_DMA skip in qcom_iommu_attach_dev), so dropping it
	 * removes a dead, blocking domain rather than any working translation.
	 */
	if (type != IOMMU_DOMAIN_UNMANAGED)
		return NULL;

	qcom_domain = kzalloc(sizeof(*qcom_domain), GFP_KERNEL);
	if (!qcom_domain)
		return NULL;

	mutex_init(&qcom_domain->init_mutex);
	spin_lock_init(&qcom_domain->pgtbl_lock);
	qcom_domain->domain.pgsize_bitmap = SZ_4K | SZ_2M | SZ_1G;

	return &qcom_domain->domain;
}

static void qcom_iommu_domain_free(struct iommu_domain *domain)
{
	struct qcom_iommu_domain *qcom_domain = to_qcom_iommu_domain(domain);

	if (qcom_domain->iommu) {
		pm_runtime_get_sync(qcom_domain->iommu->dev);
		free_io_pgtable_ops(qcom_domain->pgtbl_ops);
		pm_runtime_put_sync(qcom_domain->iommu->dev);
	}

	kfree(qcom_domain);
}

static int qcom_iommu_attach_dev(struct iommu_domain *domain,
				 struct device *dev)
{
	struct iommu_fwspec *fwspec = dev->iommu_fwspec;
	struct qcom_iommu_dev *qcom_iommu;
	struct qcom_iommu_domain *qcom_domain = to_qcom_iommu_domain(domain);
	struct platform_device *iommu_pdev;
	unsigned int i;
	int ret;

	if (!fwspec || fwspec->ops != &qcom_iommu_ops)
		return -ENXIO;

	if (!fwspec->iommu_priv) {
		/*
		 * of_xlate() ran before qcom_iommu probed (IOMMU_OF_DECLARE
		 * path) and intentionally deferred driver-data resolution.
		 * Look it up now that probe has completed.
		 */
		iommu_pdev = of_find_device_by_node(to_of_node(fwspec->iommu_fwnode));
		if (!iommu_pdev)
			return -ENODEV;
		qcom_iommu = platform_get_drvdata(iommu_pdev);
		put_device(&iommu_pdev->dev);
		if (!qcom_iommu)
			return -EPROBE_DEFER;

		for (i = 0; i < fwspec->num_ids; i++) {
			if (fwspec->ids[i] > qcom_iommu->max_asid ||
			    !qcom_iommu->ctxs[fwspec->ids[i]]) {
				dev_err(dev, "invalid asid %u\n",
					fwspec->ids[i]);
				return -EINVAL;
			}
		}
		fwspec->iommu_priv = qcom_iommu;
	}
	qcom_iommu = fwspec->iommu_priv;

	/*
	 * The IOMMU core auto-attaches a default IOMMU_DOMAIN_DMA domain when
	 * the device is added. For the GPU SMMU that happens during KGSL's
	 * of_platform_populate() at probe, long before KGSL powers the GPU, so
	 * programming the context bank here is an external abort (the CB is
	 * dead until KGSL brings the GPU rail/clocks/reset up). The functional
	 * attach is KGSL's explicit IOMMU_DOMAIN_UNMANAGED attach in
	 * kgsl_mmu_start(), which runs with the GPU powered. Non-GPU consumers
	 * (mdss/vidc/cam) likewise drive their own UNMANAGED domains, so the
	 * default DMA domain is unused on this SMMU -> skip it.
	 */
	if (domain->type == IOMMU_DOMAIN_DMA)
		return 0;

	pm_runtime_get_sync(qcom_iommu->dev);
	ret = qcom_iommu_init_domain(domain, qcom_iommu, dev);
	if (ret < 0) {
		pm_runtime_put_sync(qcom_iommu->dev);
		return ret;
	}

	/*
	 * For a power-managed SMMU (the GPU; identified by having a vdd rail),
	 * keep it powered for the lifetime of the attachment instead of
	 * dropping power here. KGSL programs the context-bank registers
	 * directly (kgsl_iommu_start -> KGSL_IOMMU_GET/SET_CTX_REG) right after
	 * this attach; power-collapsing the SMMU in between resets the
	 * TZ-restored CB security state, so that direct access faults with an
	 * external abort. The matching put is in qcom_iommu_detach_dev().
	 * Always-on SMMUs (no vdd, e.g. apps) keep the balanced get/put.
	 */
	if (!qcom_iommu->vdd)
		pm_runtime_put_sync(qcom_iommu->dev);

	/* Sanity check: we don't support domains across different IOMMUs */
	if (qcom_domain->iommu != qcom_iommu) {
		if (qcom_iommu->vdd)
			pm_runtime_put_sync(qcom_iommu->dev);
		return -EINVAL;
	}

	return 0;
}

static void qcom_iommu_detach_dev(struct iommu_domain *domain,
				  struct device *dev)
{
	struct qcom_iommu_domain *qcom_domain = to_qcom_iommu_domain(domain);
	struct iommu_fwspec *fwspec = dev->iommu_fwspec;
	struct qcom_iommu_dev *qcom_iommu = fwspec->iommu_priv;
	unsigned int i;

	/* Default DMA domain was never programmed (see attach_dev) -> nothing to do */
	if (domain->type == IOMMU_DOMAIN_DMA)
		return;

	if (WARN_ON(!qcom_domain->iommu))
		return;

	pm_runtime_get_sync(qcom_iommu->dev);
	for (i = 0; i < fwspec->num_ids; i++) {
		struct qcom_iommu_ctx *ctx = to_ctx(qcom_domain,
						    fwspec->ids[i]);
		/* Disable the context bank */
		ctx_writel(ctx, ARM_SMMU_CB_SCTLR, 0);
		ctx->domain = NULL;
	}
	pm_runtime_put_sync(qcom_iommu->dev);

	/* Release the power ref held since attach for a power-managed SMMU */
	if (qcom_iommu->vdd)
		pm_runtime_put_sync(qcom_iommu->dev);
}

static int qcom_iommu_map(struct iommu_domain *domain, unsigned long iova,
			  phys_addr_t paddr, size_t size, int prot)
{
	int ret;
	unsigned long flags;
	struct qcom_iommu_domain *qcom_domain = to_qcom_iommu_domain(domain);
	struct io_pgtable_ops *ops = qcom_domain->pgtbl_ops;

	if (!ops)
		return -ENODEV;

	spin_lock_irqsave(&qcom_domain->pgtbl_lock, flags);
	ret = ops->map(ops, iova, paddr, size, prot);
	spin_unlock_irqrestore(&qcom_domain->pgtbl_lock, flags);
	return ret;
}

static size_t qcom_iommu_unmap(struct iommu_domain *domain, unsigned long iova,
			       size_t size)
{
	size_t ret;
	unsigned long flags;
	struct qcom_iommu_domain *qcom_domain = to_qcom_iommu_domain(domain);
	struct io_pgtable_ops *ops = qcom_domain->pgtbl_ops;

	if (!ops)
		return 0;

	pm_runtime_get_sync(qcom_domain->iommu->dev);
	spin_lock_irqsave(&qcom_domain->pgtbl_lock, flags);
	ret = ops->unmap(ops, iova, size);
	spin_unlock_irqrestore(&qcom_domain->pgtbl_lock, flags);
	pm_runtime_put_sync(qcom_domain->iommu->dev);

	return ret;
}

static phys_addr_t qcom_iommu_iova_to_phys(struct iommu_domain *domain,
					    dma_addr_t iova)
{
	phys_addr_t ret;
	unsigned long flags;
	struct qcom_iommu_domain *qcom_domain = to_qcom_iommu_domain(domain);
	struct io_pgtable_ops *ops = qcom_domain->pgtbl_ops;

	if (!ops)
		return 0;

	spin_lock_irqsave(&qcom_domain->pgtbl_lock, flags);
	ret = ops->iova_to_phys(ops, iova);
	spin_unlock_irqrestore(&qcom_domain->pgtbl_lock, flags);

	return ret;
}

static bool qcom_iommu_capable(enum iommu_cap cap)
{
	switch (cap) {
	case IOMMU_CAP_CACHE_COHERENCY:
		return true;
	case IOMMU_CAP_NOEXEC:
		return true;
	default:
		return false;
	}
}

/* ------------------------------------------------------------------ */
/* Device add/remove (4.9 API)                                        */
/* ------------------------------------------------------------------ */
static int qcom_iommu_of_xlate(struct device *dev,
			       struct of_phandle_args *args)
{
	struct platform_device *iommu_pdev;
	unsigned int asid = args->args[0];

	if (args->args_count != 1) {
		dev_err(dev, "incorrect number of iommu params (%d, expected 1)\n",
			args->args_count);
		return -EINVAL;
	}

	/*
	 * Verify the IOMMU platform device exists. Driver data validation
	 * (max_asid, ctxs[]) is deferred to attach_dev() because this
	 * function may be called before qcom_iommu_device_probe() completes
	 * when IOMMU_OF_DECLARE pre-registers the ops.
	 */
	iommu_pdev = of_find_device_by_node(args->np);
	if (!iommu_pdev) {
		dev_err(dev, "IOMMU device node not registered\n");
		return -ENODEV;
	}
	put_device(&iommu_pdev->dev);

	return iommu_fwspec_add_ids(dev, &asid, 1);
}

static int qcom_iommu_add_device(struct device *dev)
{
	struct iommu_fwspec *fwspec = dev->iommu_fwspec;
	struct iommu_group *group;

	if (!fwspec || fwspec->ops != &qcom_iommu_ops)
		return -ENODEV;

	group = iommu_group_get_for_dev(dev);
	if (IS_ERR(group))
		return PTR_ERR(group);

	iommu_group_put(group);

	return 0;
}

static void qcom_iommu_remove_device(struct device *dev)
{
	struct iommu_fwspec *fwspec = dev->iommu_fwspec;

	if (!fwspec || fwspec->ops != &qcom_iommu_ops)
		return;

	iommu_group_remove_device(dev);
	iommu_fwspec_free(dev);
}

static struct iommu_group *qcom_iommu_device_group(struct device *dev)
{
	return generic_device_group(dev);
}

/*
 * KGSL (Adreno GPU) runs in global-pagetable mode on this device and
 * does not self-program the context bank (the TZ blocks HLOS access to
 * the SMMU global registers).  It only needs to read back a few CB
 * parameters so it can locate the same CB and pagetable that qcom_iommu
 * already programmed at attach time.  Expose those here.
 */
static int qcom_iommu_domain_get_attr(struct iommu_domain *domain,
				      enum iommu_attr attr, void *data)
{
	struct qcom_iommu_domain *qcom_domain = to_qcom_iommu_domain(domain);
	struct iommu_fwspec *fwspec = qcom_domain->fwspec;
	struct qcom_iommu_ctx *ctx;

	switch (attr) {
	case DOMAIN_ATTR_CONTEXT_BANK:
		if (!fwspec || !qcom_domain->iommu)
			return -ENODEV;
		ctx = to_ctx(qcom_domain, fwspec->ids[0]);
		if (!ctx)
			return -ENODEV;
		*((unsigned int *)data) = ctx->cb;
		return 0;
	case DOMAIN_ATTR_TTBR0:
		if (!qcom_domain->iommu)
			return -ENODEV;
		*((u64 *)data) = qcom_domain->ttbr0;
		return 0;
	case DOMAIN_ATTR_CONTEXTIDR:
		/* CONTEXTIDR is unused in global-pagetable mode */
		*((u32 *)data) = 0;
		return 0;
	default:
		return -EINVAL;
	}
}

static int qcom_iommu_domain_set_attr(struct iommu_domain *domain,
				      enum iommu_attr attr, void *data)
{
	struct qcom_iommu_domain *qcom_domain = to_qcom_iommu_domain(domain);
	struct iommu_fwspec *fwspec = qcom_domain->fwspec;
	struct qcom_iommu_ctx *ctx;
	u32 reg;
	int i;

	switch (attr) {
	/*
	 * Accept (no-op) the hints KGSL sets in global mode so it does not
	 * log spurious warnings; per-process attrs are never set because
	 * the GPU SMMU node declares qcom,global_pt.
	 */
	case DOMAIN_ATTR_USE_UPSTREAM_HINT:
		return 0;
	/*
	 * vidc sets the secure VMID on its secure context banks
	 * (msm_vidc_setup_context_bank) and treats a failure as fatal,
	 * dropping the CB from res->context_banks — which then feeds zeros
	 * to TZBSP_MEM_PROTECT_VIDEO_VAR and TZ rejects the protect call.
	 * Our "-sec" contexts are TZ-owned (secured_ctx: no HLOS CB writes,
	 * mappings are TZ's business), so the VMID is meaningless here;
	 * accept and ignore it. Secure (DRM) sessions are not supported
	 * either way — this only keeps the CB metadata alive for protect.
	 */
	case DOMAIN_ATTR_SECURE_VMID:
		return 0;
	/*
	 * Camera VFE/CPP requests terminate-on-fault (msm_isp_smmu_attach /
	 * msm_cpp -> cam_smmu_set_attr). It is called BEFORE cam_smmu_ops
	 * (CAM_SMMU_ATTACH), i.e. while the domain is still DETACHED and
	 * qcom_domain->iommu/fwspec are NULL, so just record the flag here;
	 * qcom_iommu_init_domain() applies it (clears SCTLR.CFCFG) when the CBs
	 * are programmed at attach time. If already attached, also clear CFCFG
	 * in place so the change takes effect immediately.
	 *
	 * In stall mode a faulting VFE write holds the AXI/CAMSS bus grant
	 * forever, wedging i2c/mdss/mmc and hanging the whole SoC (measured on
	 * x103f: fsr=0x40000202 has FSR_SS set).
	 */
	case DOMAIN_ATTR_CB_STALL_DISABLE:
		qcom_domain->cb_stall_disable = true;
		if (fwspec && qcom_domain->iommu) {
			for (i = 0; i < fwspec->num_ids; i++) {
				ctx = to_ctx(qcom_domain, fwspec->ids[i]);
				if (!ctx || ctx->secured_ctx)
					continue;
				reg = ctx_readl(ctx, ARM_SMMU_CB_SCTLR);
				reg &= ~SCTLR_CFCFG;
				ctx_writel(ctx, ARM_SMMU_CB_SCTLR, reg);
			}
		}
		return 0;
	default:
		return -EINVAL;
	}
}

static struct iommu_ops qcom_iommu_ops = {
	.capable	= qcom_iommu_capable,
	.domain_alloc	= qcom_iommu_domain_alloc,
	.domain_free	= qcom_iommu_domain_free,
	.attach_dev	= qcom_iommu_attach_dev,
	.detach_dev	= qcom_iommu_detach_dev,
	.map		= qcom_iommu_map,
	.unmap		= qcom_iommu_unmap,
	.map_sg		= default_iommu_map_sg,
	.iova_to_phys	= qcom_iommu_iova_to_phys,
	.add_device	= qcom_iommu_add_device,
	.remove_device	= qcom_iommu_remove_device,
	.device_group	= qcom_iommu_device_group,
	.domain_get_attr = qcom_iommu_domain_get_attr,
	.domain_set_attr = qcom_iommu_domain_set_attr,
	.of_xlate	= qcom_iommu_of_xlate,
	.pgsize_bitmap	= SZ_4K | SZ_2M | SZ_1G,
};

/*
 * Pre-register IOMMU ops at postcore_initcall_sync so that
 * of_iommu_configure() can resolve our ops when consumer devices are
 * created (before qcom_iommu_device_probe runs). Driver-data validation
 * is deferred to attach_dev().
 */
static int __init qcom_iommu_of_init(struct device_node *np)
{
	of_iommu_set_ops(np, &qcom_iommu_ops);
	return 0;
}

IOMMU_OF_DECLARE(qcom_iommu_v1, "qcom,msm-iommu-v1", qcom_iommu_of_init);
IOMMU_OF_DECLARE(qcom_iommu_v2, "qcom,msm-iommu-v2", qcom_iommu_of_init);

/* ------------------------------------------------------------------ */
/* Context-bank child driver                                          */
/* ------------------------------------------------------------------ */
static int get_asid(const struct device_node *np)
{
	u32 reg, val;

	if (of_property_read_u32_index(np, "reg", 0, &reg))
		return -ENODEV;

	if (!of_property_read_u32(np, "qcom,ctx-asid", &val))
		return val;

	return reg / 0x1000;
}

static int qcom_iommu_ctx_probe(struct platform_device *pdev)
{
	struct qcom_iommu_ctx *ctx;
	struct device *dev = &pdev->dev;
	struct qcom_iommu_dev *qcom_iommu = dev_get_drvdata(dev->parent);
	struct resource *res;
	int ret, irq;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dev = dev;
	platform_set_drvdata(pdev, ctx);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ctx->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(ctx->base))
		return PTR_ERR(ctx->base);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	if (of_device_is_compatible(dev->of_node, "qcom,msm-iommu-v1-sec") ||
	    of_device_is_compatible(dev->of_node, "qcom,msm-iommu-v2-sec"))
		ctx->secured_ctx = true;

	/*
	 * Do NOT touch the CB registers here. ctx_probe runs at boot, but the
	 * GPU SMMU's CB lives in the GPU GDSC which is OFF until KGSL powers it
	 * on -> a register read here is an external abort (boot panic). The CB
	 * is fully cleared/programmed in qcom_iommu_attach_dev(), which wraps
	 * its accesses in pm_runtime_get_sync() (powers the rail + clocks).
	 */

	/*
	 * Request the context-fault IRQ THREADED (handler = NULL), matching
	 * arm-smmu.c's arm_smmu_context_fault.  The camera registers a fault
	 * handler (cam_smmu_iommu_fault_handler) via iommu_set_fault_handler
	 * that takes mutexes — it is NOT IRQ-atomic.  Running qcom_iommu_fault
	 * as a hardirq invoked that handler in atomic context and blew up with
	 * "BUG: scheduling while atomic" (mutex_lock inside __schedule) the
	 * moment the VFE hit a terminate-mode fault (measured on x103f after
	 * disabling CB stall).  Threading the IRQ restores the process context
	 * the handler expects.
	 */
	ret = devm_request_threaded_irq(dev, irq, NULL, qcom_iommu_fault,
					IRQF_ONESHOT | IRQF_SHARED,
					"qcom-iommu-fault", ctx);
	if (ret) {
		dev_err(dev, "failed to request IRQ %u\n", irq);
		return ret;
	}

	ret = get_asid(dev->of_node);
	if (ret < 0) {
		dev_err(dev, "missing reg property\n");
		return ret;
	}

	ctx->asid = ret;

	/*
	 * HW context-bank index, derived from the register offset within
	 * the SMMU CB space (reg / 0x1000). This is independent of the
	 * (possibly remapped) asid and is exported to KGSL via
	 * DOMAIN_ATTR_CONTEXT_BANK so it can locate the same CB registers.
	 */
	{
		u32 reg_off = 0;

		of_property_read_u32_index(dev->of_node, "reg", 0, &reg_off);
		ctx->cb = reg_off / 0x1000;
	}

	dev_dbg(dev, "found asid %u (cb %u)\n", ctx->asid, ctx->cb);

	qcom_iommu->ctxs[ctx->asid] = ctx;

	return 0;
}

static int qcom_iommu_ctx_remove(struct platform_device *pdev)
{
	struct qcom_iommu_dev *qcom_iommu = dev_get_drvdata(pdev->dev.parent);
	struct qcom_iommu_ctx *ctx = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	qcom_iommu->ctxs[ctx->asid] = NULL;

	return 0;
}

static const struct of_device_id ctx_of_match[] = {
	{ .compatible = "qcom,msm-iommu-v1-ns" },
	{ .compatible = "qcom,msm-iommu-v1-sec" },
	{ .compatible = "qcom,msm-iommu-v2-ns" },
	{ .compatible = "qcom,msm-iommu-v2-sec" },
	{ /* sentinel */ }
};

static struct platform_driver qcom_iommu_ctx_driver = {
	.driver	= {
		.name		= "qcom-iommu-ctx",
		.of_match_table	= ctx_of_match,
	},
	.probe	= qcom_iommu_ctx_probe,
	.remove	= qcom_iommu_ctx_remove,
};

/* ------------------------------------------------------------------ */
/* Main IOMMU device driver                                           */
/* ------------------------------------------------------------------ */
static int qcom_iommu_device_probe(struct platform_device *pdev)
{
	struct device_node *child;
	struct qcom_iommu_dev *qcom_iommu;
	struct device *dev = &pdev->dev;
	struct resource *res;
	int ret, max_asid = 0;
	size_t alloc_size;

	/* Find the max asid (1:1 to ctx bank idx) */
	for_each_child_of_node(dev->of_node, child)
		max_asid = max(max_asid, get_asid(child));

	alloc_size = sizeof(*qcom_iommu) +
		     sizeof(qcom_iommu->ctxs[0]) * (max_asid + 1);
	qcom_iommu = devm_kzalloc(dev, alloc_size, GFP_KERNEL);
	if (!qcom_iommu)
		return -ENOMEM;

	qcom_iommu->max_asid = max_asid;
	qcom_iommu->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res) {
		qcom_iommu->local_base = devm_ioremap_resource(dev, res);
		if (IS_ERR(qcom_iommu->local_base))
			return PTR_ERR(qcom_iommu->local_base);
	}

	qcom_iommu->clk_iface = devm_clk_get(dev, "iface");
	if (IS_ERR(qcom_iommu->clk_iface)) {
		dev_err(dev, "failed to get iface clock\n");
		return PTR_ERR(qcom_iommu->clk_iface);
	}

	qcom_iommu->clk_bus = devm_clk_get(dev, "bus");
	if (IS_ERR(qcom_iommu->clk_bus)) {
		dev_err(dev, "failed to get bus clock\n");
		return PTR_ERR(qcom_iommu->clk_bus);
	}

	/*
	 * Optional power rail. The GPU SMMU (kgsl_smmu) lives in the GPU
	 * (oxili_gx) GDSC; its CB registers are dead unless that rail is on.
	 * The regulator framework is refcounted, so enabling it here coexists
	 * with KGSL, which is the rail's primary owner and drives the GX
	 * power-on sequence. apps_iommu has no "vdd" supply -> NULL.
	 */
	qcom_iommu->vdd = devm_regulator_get_optional(dev, "vdd");
	if (IS_ERR(qcom_iommu->vdd)) {
		if (PTR_ERR(qcom_iommu->vdd) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		qcom_iommu->vdd = NULL;
	}

	if (of_property_read_u32(dev->of_node, "qcom,iommu-secure-id",
				 &qcom_iommu->sec_id)) {
		dev_err(dev, "missing qcom,iommu-secure-id property\n");
		return -ENODEV;
	}

	platform_set_drvdata(pdev, qcom_iommu);

	pm_runtime_enable(dev);

	/* Register context bank devices (child nodes) */
	ret = of_platform_populate(dev->of_node, ctx_of_match, NULL, dev);
	if (ret) {
		dev_err(dev, "Failed to populate iommu contexts\n");
		goto err_pm_disable;
	}

	/* Route context-bank IRQs to non-secure */
	if (qcom_iommu->local_base) {
		pm_runtime_get_sync(dev);
		writel_relaxed(0xffffffff,
			       qcom_iommu->local_base + SMMU_INTR_SEL_NS);
		pm_runtime_put_sync(dev);
	}

	/* Register IOMMU with the bus */
	bus_set_iommu(&platform_bus_type, &qcom_iommu_ops);

	dev_info(dev, "qcom-iommu probed (sec_id=%u, max_asid=%u)\n",
		 qcom_iommu->sec_id, qcom_iommu->max_asid);

	return 0;

err_pm_disable:
	pm_runtime_disable(dev);
	return ret;
}

static int qcom_iommu_device_remove(struct platform_device *pdev)
{
	pm_runtime_force_suspend(&pdev->dev);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

/* ------------------------------------------------------------------ */
/* PM runtime — enable/disable clocks                                 */
/* ------------------------------------------------------------------ */
static int qcom_iommu_resume(struct device *dev)
{
	struct qcom_iommu_dev *qcom_iommu = dev_get_drvdata(dev);
	int ret;

	/* Power rail first (GPU GDSC) so the CB regs are alive for the clocks */
	if (qcom_iommu->vdd) {
		ret = regulator_enable(qcom_iommu->vdd);
		if (ret)
			return ret;
	}

	ret = clk_prepare_enable(qcom_iommu->clk_iface);
	if (ret)
		goto err_vdd;

	ret = clk_prepare_enable(qcom_iommu->clk_bus);
	if (ret) {
		clk_disable_unprepare(qcom_iommu->clk_iface);
		goto err_vdd;
	}

	return 0;

err_vdd:
	if (qcom_iommu->vdd)
		regulator_disable(qcom_iommu->vdd);
	return ret;
}

static int qcom_iommu_suspend(struct device *dev)
{
	struct qcom_iommu_dev *qcom_iommu = dev_get_drvdata(dev);

	clk_disable_unprepare(qcom_iommu->clk_bus);
	clk_disable_unprepare(qcom_iommu->clk_iface);
	if (qcom_iommu->vdd)
		regulator_disable(qcom_iommu->vdd);

	return 0;
}

static const struct dev_pm_ops qcom_iommu_pm_ops = {
	SET_RUNTIME_PM_OPS(qcom_iommu_suspend, qcom_iommu_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
};

static const struct of_device_id qcom_iommu_of_match[] = {
	{ .compatible = "qcom,msm-iommu-v1" },
	{ .compatible = "qcom,msm-iommu-v2" },
	{ /* sentinel */ }
};

static struct platform_driver qcom_iommu_driver = {
	.driver	= {
		.name		= "qcom-iommu",
		.of_match_table	= qcom_iommu_of_match,
		.pm		= &qcom_iommu_pm_ops,
	},
	.probe	= qcom_iommu_device_probe,
	.remove	= qcom_iommu_device_remove,
};

static int __init qcom_iommu_init(void)
{
	int ret;

	ret = platform_driver_register(&qcom_iommu_ctx_driver);
	if (ret)
		return ret;

	ret = platform_driver_register(&qcom_iommu_driver);
	if (ret)
		platform_driver_unregister(&qcom_iommu_ctx_driver);

	return ret;
}
device_initcall(qcom_iommu_init);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("QCOM IOMMU driver for MSM8916/MSM8909 (backported)");
