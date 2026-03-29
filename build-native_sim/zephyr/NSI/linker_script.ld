SECTIONS
 {
 nsi_tasks :
 {
  __nsi_tasks_start = .;
  __nsi_PRE_BOOT_1_tasks_start = .; KEEP(*(SORT(.nsi_PRE_BOOT_1[0-9]_task))); KEEP(*(SORT(.nsi_PRE_BOOT_1[1-9][0-9]_task))); KEEP(*(SORT(.nsi_PRE_BOOT_1[1-9][0-9][0-9]_task)));
  __nsi_PRE_BOOT_2_tasks_start = .; KEEP(*(SORT(.nsi_PRE_BOOT_2[0-9]_task))); KEEP(*(SORT(.nsi_PRE_BOOT_2[1-9][0-9]_task))); KEEP(*(SORT(.nsi_PRE_BOOT_2[1-9][0-9][0-9]_task)));
  __nsi_HW_INIT_tasks_start = .; KEEP(*(SORT(.nsi_HW_INIT[0-9]_task))); KEEP(*(SORT(.nsi_HW_INIT[1-9][0-9]_task))); KEEP(*(SORT(.nsi_HW_INIT[1-9][0-9][0-9]_task)));
  __nsi_PRE_BOOT_3_tasks_start = .; KEEP(*(SORT(.nsi_PRE_BOOT_3[0-9]_task))); KEEP(*(SORT(.nsi_PRE_BOOT_3[1-9][0-9]_task))); KEEP(*(SORT(.nsi_PRE_BOOT_3[1-9][0-9][0-9]_task)));
  __nsi_FIRST_SLEEP_tasks_start = .; KEEP(*(SORT(.nsi_FIRST_SLEEP[0-9]_task))); KEEP(*(SORT(.nsi_FIRST_SLEEP[1-9][0-9]_task))); KEEP(*(SORT(.nsi_FIRST_SLEEP[1-9][0-9][0-9]_task)));
  __nsi_ON_EXIT_PRE_tasks_start = .; KEEP(*(SORT(.nsi_ON_EXIT_PRE[0-9]_task))); KEEP(*(SORT(.nsi_ON_EXIT_PRE[1-9][0-9]_task))); KEEP(*(SORT(.nsi_ON_EXIT_PRE[1-9][0-9][0-9]_task)));
  __nsi_ON_EXIT_POST_tasks_start = .; KEEP(*(SORT(.nsi_ON_EXIT_POST[0-9]_task))); KEEP(*(SORT(.nsi_ON_EXIT_POST[1-9][0-9]_task))); KEEP(*(SORT(.nsi_ON_EXIT_POST[1-9][0-9][0-9]_task)));
  __nsi_tasks_end = .;
 }
 nsi_hw_events :
 {
  __nsi_hw_events_start = .;
  KEEP(*(SORT(.nsi_hw_event_[0-9]))); KEEP(*(SORT(.nsi_hw_event_[1-9][0-9]))); KEEP(*(SORT(.nsi_hw_event_[1-9][0-9][0-9])));
  __nsi_hw_events_end = .;
 }
 } INSERT AFTER .data;
