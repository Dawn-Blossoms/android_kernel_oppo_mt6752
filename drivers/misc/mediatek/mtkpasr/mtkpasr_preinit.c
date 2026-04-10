#define pr_fmt(fmt) "["KBUILD_MODNAME"]" fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/memory.h>
#include <linux/memblock.h>
#include <linux/printk.h>
#include <linux/sched.h>
#include <mach/emi_mpu.h>
#include <mach/mt_lpae.h>

#define CONFIG_MTKPASR_MINDIESIZE_PFN		(0x20000)	/* 512MB */
#define MTKPASR_1GB_PFNS			(0x40000)	/* 1GB */
#define MTKPASR_2GB_PFNS			(0x80000)	/* 2GB */
#define MTKPASR_3GB_PFNS			(0xC0000)	/* 3GB */
#define MTKPASR_4GB_PFNS			(0x100000)	/* 4GB */
#define MTKPASR_DRAM_MINSIZE			MTKPASR_2GB_PFNS

#define MTKPASR_INVALID_TAG			(0xEFFFFFFF)

/* #define NO_UART_CONSOLE */
#ifndef NO_UART_CONSOLE
#define PRINT(len, string, args...)	printk(KERN_ALERT string, ##args)
#else
unsigned char mtkpasr_log_buf[4096];
static int log_stored;
#define PRINT(len, string, args...)	do { sprintf(mtkpasr_log_buf + log_stored, string, ##args); log_stored += len; } while (0)
#endif

/* Reserved possible PASR range */
struct view_pasr {
	unsigned long start_pfn;	/* The 1st pfn */
	unsigned long end_pfn;		/* The pfn after the last valid one */
	unsigned long valid_start_pfn;	/* Actual PASR-masked start_pfn */
	unsigned long valid_end_pfn;	/* The pfn after the last PASR-masked one */
};
static struct view_pasr rp_pasr_info[2];

/* Struct for parsing rank information (SW view) */
struct view_rank {
	unsigned long start_pfn;	/* The 1st pfn */
	unsigned long end_pfn;		/* The pfn after the last valid one */
	unsigned long bank_pfn_size;	/* Bank size in PFN */
	unsigned long valid_channel;	/* Channels: 0x00000101 means there are 2 valid channels - 1st & 2nd (MAX: 4 channels) */
};
static struct view_rank rank_info[MAX_RANKS];

#define SEGMENTS_PER_RANK	(8)

/* Basic DRAM configuration */
static struct basic_dram_setting pasrdpd;
/* PASR/DPD imposed start rank */
static int mtkpasr_start_rank;
/* MAX Banksize in pfns (from SW view) */
unsigned long pasrbank_pfns = 0;
/*
 * We can't guarantee HIGHMEM zone is bank alignment, so we need another variable to represent it.
 * (mtkpasr_pfn_start, mtkpasr_pfn_end) is bank-alignment!
 */
static unsigned long mtkpasr_pfn_start;
static unsigned long mtkpasr_pfn_end;

/* Segment mask */
static unsigned long valid_segment = 0x0;

/* Set pageblock's mobility */
extern void set_pageblock_mobility(struct page *page, int mobility);

/* From dram_overclock.c */
extern bool pasr_is_valid(void)__attribute__((weak));
/* To confirm PASR is valid */
static inline bool could_do_mtkpasr(void)
{
	if (mtkpasr_start_rank == MTKPASR_INVALID_TAG)
		return false;

	if (pasr_is_valid)
		return pasr_is_valid();

	return false;
}

#define MAX_RANK_PFN	(0x1FFFFF)
#define MAX_KERNEL_PFN	(0x13FFFF)
#define MAX_KPFN_MASK	(0x0FFFFF)
#define KPFN_TO_VIRT	(0x100000)
static unsigned long __init virt_to_kernel_pfn(unsigned long virt)
{
	unsigned long ret = virt;

	if (enable_4G()) {
		if (virt > MAX_RANK_PFN)
			ret = virt - KPFN_TO_VIRT;
		else if (virt > MAX_KERNEL_PFN)
			ret = virt & MAX_KPFN_MASK;
	}

	return ret;
}
static unsigned long __init kernel_pfn_to_virt(unsigned long kpfn)
{
	unsigned long ret = kpfn;

	if (enable_4G())
		ret = kpfn | KPFN_TO_VIRT;

	return ret;
}
static unsigned long __init rank_pfn_offset(void)
{
	unsigned long ret = ARCH_PFN_OFFSET;

	if (enable_4G())
		ret = KPFN_TO_VIRT;

	return ret;
}

/*
 * Parse DRAM setting - transform DRAM setting to temporary bank structure.
 */
extern void acquire_dram_setting(struct basic_dram_setting *pasrdpd)__attribute__((weak));
static bool __init parse_dram_setting(unsigned long hint)
{
	int channel_num, chan, rank, check_segment_num;
	unsigned long valid_channel;
	unsigned long check_rank_size, rank_pfn, start_pfn = rank_pfn_offset();

	PRINT(29, "rank_pfn_offset() [0x%8lx]\n", rank_pfn_offset());

	if (acquire_dram_setting) {
		hint = 0;
		acquire_dram_setting(&pasrdpd);
		channel_num = pasrdpd.channel_nr;
		/* By ranks */
		for (rank = 0; rank < MAX_RANKS; ++rank) {
			rank_pfn = 0;
			rank_info[rank].valid_channel = 0x0;
			valid_channel = 0x1;
			check_rank_size = 0x0;
			check_segment_num = 0x0;
			for (chan = 0; chan < channel_num; ++chan) {
				if (pasrdpd.channel[chan].rank[rank].valid_rank) {
					rank_pfn += (pasrdpd.channel[chan].rank[rank].rank_size << (27 - PAGE_SHIFT));
					rank_info[rank].valid_channel |= valid_channel;
					/* Sanity check for rank size */
					if (!check_rank_size) {
						check_rank_size = pasrdpd.channel[chan].rank[rank].rank_size;
					} else {
						/* We only support ranks with equal size */
						if (check_rank_size != pasrdpd.channel[chan].rank[rank].rank_size) {
							return false;
						}
					}
					/* Sanity check for segment number */
					if (!check_segment_num) {
						check_segment_num = pasrdpd.channel[chan].rank[rank].segment_nr;
					} else {
						/* We only support ranks with equal segment number */
						if (check_segment_num != pasrdpd.channel[chan].rank[rank].segment_nr) {
							return false;
						}
					}
				}
				valid_channel <<= 8;
			}
			/* Have we found a valid rank */
			if (check_rank_size != 0 && check_segment_num != 0) {
				rank_info[rank].start_pfn = virt_to_kernel_pfn(start_pfn);
				rank_info[rank].end_pfn = virt_to_kernel_pfn(start_pfn + rank_pfn);
				rank_info[rank].bank_pfn_size = rank_pfn/check_segment_num;
				start_pfn = kernel_pfn_to_virt(rank_info[rank].end_pfn);
				PRINT(96, "Rank[%d] start_pfn[%8lu] end_pfn[%8lu] bank_pfn_size[%8lu] valid_channel[0x%-8lx]\n",
						rank, rank_info[rank].start_pfn, rank_info[rank].end_pfn,
						rank_info[rank].bank_pfn_size, rank_info[rank].valid_channel);
			} else {
				rank_info[rank].start_pfn = virt_to_kernel_pfn(rank_pfn_offset());
				rank_info[rank].end_pfn = virt_to_kernel_pfn(rank_pfn_offset());
				rank_info[rank].bank_pfn_size = 0;
				rank_info[rank].valid_channel = 0x0;
			}
			/* Calculate total pfns */
			hint += rank_pfn;
		}
	} else {
		/* Single channel, dual ranks, 8 segments per rank - Get a hint from system */
		rank_pfn = (hint + CONFIG_MTKPASR_MINDIESIZE_PFN - 1) & ~(CONFIG_MTKPASR_MINDIESIZE_PFN - 1);
		rank_pfn >>= 1;
		for (rank = 0; rank < 2; ++rank) {
			rank_info[rank].start_pfn = virt_to_kernel_pfn(start_pfn);
			rank_info[rank].end_pfn = virt_to_kernel_pfn(start_pfn + rank_pfn);
			rank_info[rank].bank_pfn_size = rank_pfn >> 3;
			rank_info[rank].valid_channel = 0x1;
			start_pfn = kernel_pfn_to_virt(rank_info[rank].end_pfn);
			PRINT(96, "(--)Rank[%d] start_pfn[%8lu] end_pfn[%8lu] bank_pfn_size[%8lu] valid_channel[0x%-8lx]\n",
					rank, rank_info[rank].start_pfn, rank_info[rank].end_pfn,
					rank_info[rank].bank_pfn_size, rank_info[rank].valid_channel);
		}
		/* Reset remaining ranks */
		for (; rank < MAX_RANKS; ++rank) {
			rank_info[rank].start_pfn = virt_to_kernel_pfn(rank_pfn_offset());
			rank_info[rank].end_pfn = virt_to_kernel_pfn(rank_pfn_offset());
			rank_info[rank].bank_pfn_size = 0;
			rank_info[rank].valid_channel = 0x0;
		}
	}

	/* Check whether it is suitable to enable PASR */
	if (hint < MTKPASR_DRAM_MINSIZE) {
		printk(KERN_ALERT "[MTKPASR] Total memory: %lu < 1GB\n", (hint << PAGE_SHIFT));
		return false;
	}

	return true;
}

/* Check whether it is a valid rank */
static bool __init is_valid_rank(int rank)
{
	/* Check start/end pfn */
	if (rank_info[rank].start_pfn == rank_info[rank].end_pfn) {
		return false;
	}

	/* Check valid_channel */
	if (rank_info[rank].valid_channel == 0x0) {
		return false;
	}

	return true;
}

#define PHYS_TO_PFN(x)	__phys_to_pfn(x)
#define PHYS_TO_PFN_ROUND_DOWN(x, y)	__phys_to_pfn(x & ~(y - 1))
#define PHYS_TO_PFN_ROUND_UP(x, y)	__phys_to_pfn((x + y - 1) & ~(y - 1))
/* Fill valid_segment */
static void __init find_mtkpasr_valid_segment(unsigned long *start, unsigned long *end)
{
	int num_segment, rank;
	unsigned long spfn = 0, epfn = 0, max_start, min_end;
	unsigned long rspfn, repfn;
	unsigned long bank_pfn_size;
	bool virtual;

	/* Sanity check */
	if (*start == *end)
		return;

	num_segment = 0;
	max_start = *start;
	min_end = *end;
	for (rank = 0; rank < MAX_RANKS; ++rank, num_segment += SEGMENTS_PER_RANK) {

		/* Is it a valid rank */
		if (!is_valid_rank(rank))
			continue;

		/* At least 1 bank size  */
		bank_pfn_size = rank_info[rank].bank_pfn_size;
		if ((*start + bank_pfn_size) > *end)
			continue;

		/* If rank's start_pfn > rank's end_pfn, then compare them in virtual */
		if (rank_info[rank].start_pfn > rank_info[rank].end_pfn) {
			spfn = kernel_pfn_to_virt(*start);
			epfn = kernel_pfn_to_virt(*end);
			rspfn = kernel_pfn_to_virt(rank_info[rank].start_pfn);
			repfn = kernel_pfn_to_virt(rank_info[rank].end_pfn);
			virtual = true;
		} else {
			spfn = *start;
			epfn = *end;
			rspfn = rank_info[rank].start_pfn;
			repfn = rank_info[rank].end_pfn;
			virtual = false;
		}

		/* Update mtkpasr range */
		spfn = round_up(spfn, bank_pfn_size);
		epfn = round_down(epfn, bank_pfn_size);
		if (virtual) {
			max_start = max(virt_to_kernel_pfn(spfn), max_start);
			min_end = min(virt_to_kernel_pfn(epfn), min_end);
		} else {
			max_start = max(spfn, max_start);
			min_end = min(epfn, min_end);
		}

		/* Check overlapping */
		if (epfn <= spfn) {
			/* spfn ~ repfn */
			while (repfn >= (spfn + bank_pfn_size)) {
				valid_segment |= (1 << ((spfn - rspfn) / bank_pfn_size + num_segment));
				spfn += bank_pfn_size;
			}
			/* rspfn ~ epfn */
			while (epfn >= (rspfn + bank_pfn_size)) {
				epfn -= bank_pfn_size;
				valid_segment |= (1 << ((epfn - rspfn) / bank_pfn_size + num_segment));
			}
		} else {
			/* spfn ~ epfn */
			spfn = max(spfn, rspfn);
			epfn = min(epfn, repfn);
			while (epfn >= (spfn + bank_pfn_size)) {
				valid_segment |= (1 << ((spfn - rspfn) / bank_pfn_size + num_segment));
				spfn += bank_pfn_size;
			}
		}
	}

	/* Feedback PASR-masked range */
	if ((spfn == 0) && (epfn == 0)) {
		*start = max_start;
		*end = max_start;
	} else {
		*start = max_start;
		*end = min_end;
	}
}

static void __init update_rp_pasr_info(unsigned long spfn, unsigned long epfn)
{
	/* Check MAX range and swap them (START with (0,0)) */
	if ((epfn - spfn) > (rp_pasr_info[0].end_pfn - rp_pasr_info[0].start_pfn)) {
		rp_pasr_info[1].start_pfn = rp_pasr_info[0].start_pfn;
		rp_pasr_info[1].end_pfn = rp_pasr_info[0].end_pfn;
		rp_pasr_info[0].start_pfn = spfn;
		rp_pasr_info[0].end_pfn = epfn;
	} else if ((epfn - spfn) > (rp_pasr_info[1].end_pfn - rp_pasr_info[1].start_pfn)) { /* Sub-MAX */
		rp_pasr_info[1].start_pfn = spfn;
		rp_pasr_info[1].end_pfn = epfn;
	}
}

/* Exclude memblock.reserved */
static void __init exclude_memblock_reserved(unsigned long start, unsigned long end)
{
	struct memblock_region *rreg;
	unsigned long rstart = 0;
	unsigned long rend = ~(unsigned long)0;
	unsigned long spfn, epfn;

	/* Exclude kernel-reserved area */	
	for_each_memblock(reserved, rreg) {
		rstart = PHYS_TO_PFN_ROUND_DOWN(rreg->base, PAGE_SIZE);
		rend = PHYS_TO_PFN_ROUND_UP(rreg->base + rreg->size, PAGE_SIZE);
		
		/* start must be smaller than end */
		if (start >= end)
			break;

		/* Find available region */
		if (rstart >= start) {
			spfn = start;
			epfn = min(end, rstart);
			start = rend;
		} else {
			start = max(start, rend);
			continue;
		}

		/* Check MAX range and swap them (START with (0,0)) */
		update_rp_pasr_info(spfn, epfn);

		/* Reset */
		epfn = spfn = 0;
	}

	/* Last start & end */
	if (start < end)
		update_rp_pasr_info(start, end);
}

/* Remove the range between spfn & epfn */
static void __init __remove_needless_reserved(unsigned long spfn, unsigned long epfn)
{
	int order;
	struct list_head *curr, *tmp;
	struct page *spage;
	unsigned long flags;
	unsigned long pfn;

	if (spfn < epfn) {
		/* Search freelist */
		for (order = 0; order < MAX_ORDER; order++) {
			spin_lock_irqsave(&MTKPASR_ZONE->lock, flags);
			list_for_each_safe(curr, tmp, &MTKPASR_ZONE->free_area[order].free_list[MIGRATE_MTKPASR]) {
				spage = list_entry(curr, struct page, lru);
				pfn = page_to_pfn(spage);
				if (pfn >= spfn && pfn < epfn) {
					/* Move it from original mobility to MIGRATE_MOVABLE */
					list_move(&spage->lru, &MTKPASR_ZONE->free_area[order].free_list[MIGRATE_MOVABLE]);
					/* Set it to MIGRATE_MOVABLE */
					set_pageblock_mobility(spage, MIGRATE_MOVABLE);
				}
			}
			spin_unlock_irqrestore(&MTKPASR_ZONE->lock, flags);
		}
		spin_lock_irqsave(&MTKPASR_ZONE->lock, flags);

		/* Search inuse */
		for (pfn = spfn; pfn < epfn; pfn += pageblock_nr_pages) {
			spage = pfn_to_page(pfn);
			/* Set it to MIGRATE_MOVABLE */
			set_pageblock_mobility(spage, MIGRATE_MOVABLE);
		}
		spin_unlock_irqrestore(&MTKPASR_ZONE->lock, flags);
	}
}

/* Mark those MTKPASRed pages as MOVABLE */
static void __init remove_needless_reserved(void)
{
	int index;

	/* No PASR, remove all */
	if (mtkpasr_pfn_end == 0) {
		for (index = 0; index < 2; index++) {
			/* start_pfn ~ end_pfn */
			__remove_needless_reserved(rp_pasr_info[index].start_pfn, rp_pasr_info[index].end_pfn);
		}
	} else {
		/* Remove needless */
		for (index = 0; index < 2; index++) {
			/* start_pfn ~ valid_start_pfn */
			__remove_needless_reserved(rp_pasr_info[index].start_pfn, rp_pasr_info[index].valid_start_pfn);
			/* valid_end_pfn ~ end_pfn */
			__remove_needless_reserved(rp_pasr_info[index].valid_end_pfn, rp_pasr_info[index].end_pfn);
		}
	}
}

/*
 * Reserve a range for PASR operation. (MIGRATE_MTKPASR)
 */
void __init init_mtkpasr_range(struct zone *zone)
{
	struct memblock_region *reg;
	unsigned long start = 0;
	unsigned long end = ~(unsigned long)0;
	unsigned long min_start, max_end;
	struct page *page;

#ifdef CONFIG_HIGHMEM
	/* Start from HIGHMEM zone if we have CONFIG_HIGHMEM defined. */
	zone = zone + ZONE_HIGHMEM;
#else
	/* 64-bit kernel */
	zone = zone + ZONE_DMA;
#endif

	/* Sanity Check */
	if (zone != MTKPASR_ZONE) {
		mtkpasr_start_rank = MTKPASR_INVALID_TAG;
		return;
	}

	/* The range of MTKPASR_ZONE */
	min_start = zone->zone_start_pfn;
	max_end = zone->zone_start_pfn + zone->spanned_pages;

	/* Reserve possible PASR range */
	for_each_memblock(memory, reg) {
		start = PHYS_TO_PFN(reg->base);
		end = PHYS_TO_PFN(reg->base + reg->size);
		end = min(end, max_end);
		/* Exclude memblock reserved */
		exclude_memblock_reserved(start, end);
	}

	/* Do pre-reservation for PASR */
	for (start = 0; start < 2; start++) {
		/* Don't be out of MTKPASR_ZONE */
		if (rp_pasr_info[start].end_pfn <= min_start || rp_pasr_info[start].start_pfn >= max_end) {
			rp_pasr_info[start].start_pfn = 0;
			rp_pasr_info[start].end_pfn = 0;
			continue;
		}
		/* To fit in MTKPASR_ZONE */
		rp_pasr_info[start].start_pfn = max(min_start, rp_pasr_info[start].start_pfn);
		rp_pasr_info[start].end_pfn = min(max_end, rp_pasr_info[start].end_pfn);
		/* pageblock_nr_pages alignment */
		rp_pasr_info[start].start_pfn = (rp_pasr_info[start].start_pfn + pageblock_nr_pages - 1) & ~(pageblock_nr_pages - 1); 
		rp_pasr_info[start].end_pfn = (rp_pasr_info[start].end_pfn) & ~(pageblock_nr_pages - 1); 
		/* Mark it as MIGRATE_MTKPASR */
		for (end = rp_pasr_info[start].start_pfn; end < rp_pasr_info[start].end_pfn; end++) {
			if (!pfn_valid(end))
				continue;
			/* Set it as MIGRATE_MTKPASR - no zone lock here! (zone is not completely ready) */
			page = pfn_to_page(end);
			if (!(end & (pageblock_nr_pages - 1)))
				set_pageblock_mobility(page, MIGRATE_MTKPASR);
		}
	}
}

/*
 * We will set an offset on which active PASR will be imposed.
 * This is done by setting those pages as MIGRATE_MTKPASR type.
 * It only takes effect on HIGHMEM zone now!
 */
static bool __init initialize_mtkpasr_range(void)
{
	struct zone *zone;
	struct pglist_data *pgdat;
	int rank;
	unsigned long start = 0;
	unsigned long start_pfn, end_pfn, max_pfns;

	/* Check whether our platform supports PASR */
	if (!could_do_mtkpasr()) {
		/* Can't support PASR */
		goto recover;
	}

	/* Indicate node */
	zone = MTKPASR_ZONE;
	pgdat = zone->zone_pgdat;

	/* Parsing DRAM setting */
	if (parse_dram_setting(pgdat->node_spanned_pages) == false) {
		/* Can't support PASR */
		goto recover;
	}
	
	/* Permitted size (~ 3/8 total DRAM size) */
	max_pfns = (pgdat->node_spanned_pages * 3) >> 3;
	pr_alert("%s permitted size[%lu]\n", __func__, max_pfns);

	/* Find out valid PASR segment from rp_pasr_info */
	for (start = 0; start < 2; start++) {
		start_pfn = rp_pasr_info[start].start_pfn;
		end_pfn = rp_pasr_info[start].end_pfn;
		pr_alert(".. %s [%lu] [%lu]\n", __func__, start_pfn, end_pfn);
		/* Don't exceed permitted size */
		if ((end_pfn - start_pfn) > max_pfns)
			end_pfn = start_pfn + max_pfns;
		pr_alert("++ %s [%lu] [%lu]\n", __func__, start_pfn, end_pfn);
		find_mtkpasr_valid_segment(&start_pfn, &end_pfn);
		pr_alert("-- %s [%lu] [%lu]\n", __func__, start_pfn, end_pfn);
		if (mtkpasr_pfn_start == 0) {
			mtkpasr_pfn_start = start_pfn;
			mtkpasr_pfn_end = end_pfn;
		} else {
			mtkpasr_pfn_start = min(mtkpasr_pfn_start, start_pfn);
			mtkpasr_pfn_end = max(mtkpasr_pfn_end, end_pfn);
		}
		/* Feedback PASR-masked range */
		rp_pasr_info[start].valid_start_pfn = start_pfn;
		rp_pasr_info[start].valid_end_pfn = end_pfn;
		/* Fix max_pfns */
		max_pfns -= (end_pfn - start_pfn);
	}
	
	/* Check valid pasrbank_pfns(should be equal bank_pfn_size) */
	for (rank = 0; rank < MAX_RANKS; ++rank) {
		if (!is_valid_rank(rank))
			continue;
		pasrbank_pfns = rank_info[rank].bank_pfn_size;
	}

	PRINT(138, "[MTKPASR] @@@@@@ Start_pfn[%8lu] End_pfn[%8lu] (MTKPASR) start_pfn[%8lu] end_pfn[%8lu] Valid_segment[0x%8lx] @@@@@@\n",
			start_pfn, end_pfn, mtkpasr_pfn_start, mtkpasr_pfn_end, valid_segment);

	/* Put needless MIGRATE_MTKPASR pages back to buddy - TODO */
	remove_needless_reserved();
	return true;

recover:
	/* Recover pages with MIGRATE_MTKPASR flag to be MIGRATE_MOVABLE - TODO */
	PRINT(45, "Change page mobility from MTKPASR to MOVABLE\n");
	remove_needless_reserved();
	return false;
}

/*
 * Helper of constructing Memory (Virtual) Rank & Bank Information -
 *
 * start_pfn	  - Pfn of the 1st page in that pasr range (Should be bank-aligned)
 * end_pfn	  - Pfn of the one after the last page in that pasr range (Should be bank-aligned)
 *		    (A hole may exist between end_pfn & bank-aligned(last_valid_pfn))
 * banks_per_rank - Number of banks in a rank
 *
 * Return    - The number of memory (virtual) banks, -1 means no valid range for PASR
 */
int __init compute_valid_pasr_range(unsigned long *start_pfn, unsigned long *end_pfn, int *num_ranks)
{
	int num_banks, rank, seg_num;
	unsigned long rspfn, repfn;
	unsigned long vseg;
	bool contain_rank;

	/* Initialize MTKPASR range */
	if (!initialize_mtkpasr_range()) {
		/* Can't support PASR */
		return -1;
	}
	
	/* Bitmap for valid_segment */
	vseg = valid_segment;

	/* Set PASR/DPD range */
	*start_pfn = mtkpasr_pfn_start;
	*end_pfn = mtkpasr_pfn_end;

	/* Compute number of banks & ranks*/
	num_banks = 0;
	*num_ranks = 0;
	for (rank = 0; rank < MAX_RANKS; ++rank) {
		if (is_valid_rank(rank)) {
			contain_rank = true;
			if (rank_info[rank].start_pfn > rank_info[rank].end_pfn) {
				rspfn = kernel_pfn_to_virt(rank_info[rank].start_pfn);
				repfn = kernel_pfn_to_virt(rank_info[rank].end_pfn);
			} else {
				rspfn = rank_info[rank].start_pfn;
				repfn = rank_info[rank].end_pfn;
			}
			seg_num = (repfn - rspfn) / rank_info[rank].bank_pfn_size;
			while (seg_num--) {
				if (vseg & 0x1) {
					num_banks++;
				} else {
					contain_rank = false;
				}
				vseg >>= 1;
			}
			if (contain_rank) {
				*num_ranks += 1;
			}
		}
		vseg = valid_segment;
		vseg >>= SEGMENTS_PER_RANK;
	}

	/* No valid banks */
	if (num_banks == 0) {
		return -1;
	}

	return num_banks;
}

/*
 * Give bank, this function will return its (start_pfn, end_pfn) and corresponding rank
 * ("fully == true" means we should identify whether whole bank's rank is in a PASRDPD-imposed range)
 */
int __init query_bank_information(int bank, unsigned long *spfn, unsigned long *epfn, bool fully, int *segn)
{
	int seg_num = 0, rank, num_segment = 0;
	unsigned long vseg = valid_segment, valid_mask;
	unsigned long rspfn, repfn;
	bool virtual;

	/* Reset */
	*spfn = 0;
	*epfn = 0;

	/* Which segment */
	do {
		if (vseg & 0x1) {
			/* Found! */
			if (!bank)
				break;
			bank--;
		}
		vseg >>= 1;
		seg_num++;
	} while (seg_num < BITS_PER_LONG);

	/* Sanity check */
	if (seg_num == BITS_PER_LONG)
		return -1;

	/* Corresponding segment */
	*segn = seg_num;

	/* Which rank */
	vseg = valid_segment;
	for (rank = 0; rank < MAX_RANKS; ++rank) {
		if (is_valid_rank(rank)) {
			if (rank_info[rank].start_pfn > rank_info[rank].end_pfn) {
				rspfn = kernel_pfn_to_virt(rank_info[rank].start_pfn);
				repfn = kernel_pfn_to_virt(rank_info[rank].end_pfn);
				virtual = true;
			} else {
				rspfn = rank_info[rank].start_pfn;
				repfn = rank_info[rank].end_pfn;
				virtual = false;
			}
			num_segment = (repfn - rspfn) / rank_info[rank].bank_pfn_size;
			if (seg_num < num_segment) {
				*spfn = rank_info[rank].start_pfn + seg_num * rank_info[rank].bank_pfn_size;
				*epfn = *spfn + rank_info[rank].bank_pfn_size;
				break;
			}
			/* Next rank */
			seg_num -= SEGMENTS_PER_RANK;
			vseg >>= SEGMENTS_PER_RANK;
		}
	}
	
	/* epfn should be larger than spfn */
	if (*epfn <= *spfn)
		return -2;

	/* Sanity check */
	if (rank == MAX_RANKS)
		return -3;

	/* Convert to actual kernel pfn */
	if (virtual) {
		*spfn = virt_to_kernel_pfn(*spfn);
		*epfn = virt_to_kernel_pfn(*epfn);
	}

	/* Fix the case of *epfn < *spfn */
	if (*epfn < *spfn)
		*epfn = kernel_pfn_to_virt(*epfn);

	/* Should acquire rank information according to "rank" */
	if (fully) {
		valid_mask = (1 << num_segment) - 1;
		if ((vseg & valid_mask) == valid_mask) {
			return rank;
		}
	}

	return -1;
}
