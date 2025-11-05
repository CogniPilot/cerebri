/*
 * Copyright (c) 2025 CogniPilot Foundation
 * Copyright 2025 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(ddr), okay)
GROUP_START(DDR)

	SECTION_PROLOGUE(_IMX95_DDR_SECTION_NAME, (NOLOAD),)
	{
		*(.bss._ring_buffer_data_rb_sdcard)
	} GROUP_LINK_IN(DDR)

GROUP_END(DDR)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(ocram), okay)
GROUP_START(OCRAM)

SECTION_DATA_PROLOGUE(.kheaps_extra, (NOLOAD),)
{
    . = ALIGN(8);
    KEEP(*(.noinit.*.kheap_buf_*))
    KEEP(*(.noinit.kheap_buf_*))
    . = ALIGN(8);
} GROUP_LINK_IN(OCRAM)

GROUP_END(OCRAM)
#endif