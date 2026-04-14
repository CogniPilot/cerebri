#ifndef RDD2_HOTPATH_MEMORY_H_
#define RDD2_HOTPATH_MEMORY_H_

#include <zephyr/devicetree.h>

#if defined(CONFIG_BOARD_MR_VMU_TROPIC_MIMXRT1064) && DT_HAS_CHOSEN(zephyr_dtcm) &&                \
	DT_NODE_HAS_STATUS_OKAY(DT_CHOSEN(zephyr_dtcm))
#include <zephyr/linker/section_tags.h>
#define RDD2_HOTPATH_DTCM_BSS __dtcm_bss_section
#else
#define RDD2_HOTPATH_DTCM_BSS
#endif

#endif
