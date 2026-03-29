SECTIONS
{
rom_start :
{
}
 initlevel :
 {
  __init_start = .;
  __init_EARLY_start = .; KEEP(*(SORT(.z_init_EARLY_P_?_*))); KEEP(*(SORT(.z_init_EARLY_P_??_*))); KEEP(*(SORT(.z_init_EARLY_P_???_*)));
  __init_PRE_KERNEL_1_start = .; KEEP(*(SORT(.z_init_PRE_KERNEL_1_P_?_*))); KEEP(*(SORT(.z_init_PRE_KERNEL_1_P_??_*))); KEEP(*(SORT(.z_init_PRE_KERNEL_1_P_???_*)));
  __init_PRE_KERNEL_2_start = .; KEEP(*(SORT(.z_init_PRE_KERNEL_2_P_?_*))); KEEP(*(SORT(.z_init_PRE_KERNEL_2_P_??_*))); KEEP(*(SORT(.z_init_PRE_KERNEL_2_P_???_*)));
  __init_POST_KERNEL_start = .; KEEP(*(SORT(.z_init_POST_KERNEL_P_?_*))); KEEP(*(SORT(.z_init_POST_KERNEL_P_??_*))); KEEP(*(SORT(.z_init_POST_KERNEL_P_???_*)));
  __init_APPLICATION_start = .; KEEP(*(SORT(.z_init_APPLICATION_P_?_*))); KEEP(*(SORT(.z_init_APPLICATION_P_??_*))); KEEP(*(SORT(.z_init_APPLICATION_P_???_*)));
  __init_SMP_start = .; KEEP(*(SORT(.z_init_SMP_P_?_*))); KEEP(*(SORT(.z_init_SMP_P_??_*))); KEEP(*(SORT(.z_init_SMP_P_???_*)));
  __init_end = .;
 }
 device_area : { _device_list_start = .; KEEP(*(SORT(._device.static.*_?_*))); KEEP(*(SORT(._device.static.*_??_*))); KEEP(*(SORT(._device.static.*_???_*))); KEEP(*(SORT(._device.static.*_????_*))); KEEP(*(SORT(._device.static.*_?????_*))); _device_list_end = .;; }
 initlevel_error :
 {
  KEEP(*(SORT(.z_init_*)))
 }
 ASSERT(SIZEOF(initlevel_error) == 0, "Undefined initialization levels used.")
 app_shmem_regions :
 {
  __app_shmem_regions_start = .;
  KEEP(*(SORT(.app_regions.*)));
  __app_shmem_regions_end = .;
 }
 k_p4wq_initparam_area : { _k_p4wq_initparam_list_start = .; KEEP(*(SORT_BY_NAME(._k_p4wq_initparam.static.*))); _k_p4wq_initparam_list_end = .;; }
 _static_thread_data_area : { __static_thread_data_list_start = .; KEEP(*(SORT_BY_NAME(.__static_thread_data.static.*))); __static_thread_data_list_end = .;; }
 device_deps :
 {
__device_deps_start = .;
KEEP(*(SORT(.__device_deps_pass2*)));
__device_deps_end = .;
 }
sensor_driver_api_area : { _sensor_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._sensor_driver_api.static.*))); _sensor_driver_api_list_end = .;; }
shared_irq_driver_api_area : { _shared_irq_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._shared_irq_driver_api.static.*))); _shared_irq_driver_api_list_end = .;; }
crypto_driver_api_area : { _crypto_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._crypto_driver_api.static.*))); _crypto_driver_api_list_end = .;; }
adc_driver_api_area : { _adc_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._adc_driver_api.static.*))); _adc_driver_api_list_end = .;; }
auxdisplay_driver_api_area : { _auxdisplay_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._auxdisplay_driver_api.static.*))); _auxdisplay_driver_api_list_end = .;; }
bbram_driver_api_area : { _bbram_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._bbram_driver_api.static.*))); _bbram_driver_api_list_end = .;; }
bt_hci_driver_api_area : { _bt_hci_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._bt_hci_driver_api.static.*))); _bt_hci_driver_api_list_end = .;; }
can_driver_api_area : { _can_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._can_driver_api.static.*))); _can_driver_api_list_end = .;; }
cellular_driver_api_area : { _cellular_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._cellular_driver_api.static.*))); _cellular_driver_api_list_end = .;; }
charger_driver_api_area : { _charger_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._charger_driver_api.static.*))); _charger_driver_api_list_end = .;; }
clock_control_driver_api_area : { _clock_control_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._clock_control_driver_api.static.*))); _clock_control_driver_api_list_end = .;; }
comparator_driver_api_area : { _comparator_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._comparator_driver_api.static.*))); _comparator_driver_api_list_end = .;; }
coredump_driver_api_area : { _coredump_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._coredump_driver_api.static.*))); _coredump_driver_api_list_end = .;; }
counter_driver_api_area : { _counter_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._counter_driver_api.static.*))); _counter_driver_api_list_end = .;; }
crc_driver_api_area : { _crc_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._crc_driver_api.static.*))); _crc_driver_api_list_end = .;; }
dac_driver_api_area : { _dac_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._dac_driver_api.static.*))); _dac_driver_api_list_end = .;; }
dai_driver_api_area : { _dai_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._dai_driver_api.static.*))); _dai_driver_api_list_end = .;; }
display_driver_api_area : { _display_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._display_driver_api.static.*))); _display_driver_api_list_end = .;; }
dma_driver_api_area : { _dma_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._dma_driver_api.static.*))); _dma_driver_api_list_end = .;; }
edac_driver_api_area : { _edac_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._edac_driver_api.static.*))); _edac_driver_api_list_end = .;; }
eeprom_driver_api_area : { _eeprom_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._eeprom_driver_api.static.*))); _eeprom_driver_api_list_end = .;; }
emul_bbram_driver_api_area : { _emul_bbram_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._emul_bbram_driver_api.static.*))); _emul_bbram_driver_api_list_end = .;; }
fuel_gauge_emul_driver_api_area : { _fuel_gauge_emul_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._fuel_gauge_emul_driver_api.static.*))); _fuel_gauge_emul_driver_api_list_end = .;; }
emul_sensor_driver_api_area : { _emul_sensor_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._emul_sensor_driver_api.static.*))); _emul_sensor_driver_api_list_end = .;; }
entropy_driver_api_area : { _entropy_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._entropy_driver_api.static.*))); _entropy_driver_api_list_end = .;; }
espi_driver_api_area : { _espi_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._espi_driver_api.static.*))); _espi_driver_api_list_end = .;; }
espi_saf_driver_api_area : { _espi_saf_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._espi_saf_driver_api.static.*))); _espi_saf_driver_api_list_end = .;; }
flash_driver_api_area : { _flash_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._flash_driver_api.static.*))); _flash_driver_api_list_end = .;; }
fpga_driver_api_area : { _fpga_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._fpga_driver_api.static.*))); _fpga_driver_api_list_end = .;; }
fuel_gauge_driver_api_area : { _fuel_gauge_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._fuel_gauge_driver_api.static.*))); _fuel_gauge_driver_api_list_end = .;; }
gnss_driver_api_area : { _gnss_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._gnss_driver_api.static.*))); _gnss_driver_api_list_end = .;; }
gpio_driver_api_area : { _gpio_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._gpio_driver_api.static.*))); _gpio_driver_api_list_end = .;; }
haptics_driver_api_area : { _haptics_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._haptics_driver_api.static.*))); _haptics_driver_api_list_end = .;; }
hwspinlock_driver_api_area : { _hwspinlock_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._hwspinlock_driver_api.static.*))); _hwspinlock_driver_api_list_end = .;; }
i2c_driver_api_area : { _i2c_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._i2c_driver_api.static.*))); _i2c_driver_api_list_end = .;; }
i2c_target_driver_api_area : { _i2c_target_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._i2c_target_driver_api.static.*))); _i2c_target_driver_api_list_end = .;; }
i2s_driver_api_area : { _i2s_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._i2s_driver_api.static.*))); _i2s_driver_api_list_end = .;; }
i3c_driver_api_area : { _i3c_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._i3c_driver_api.static.*))); _i3c_driver_api_list_end = .;; }
ipm_driver_api_area : { _ipm_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._ipm_driver_api.static.*))); _ipm_driver_api_list_end = .;; }
led_driver_api_area : { _led_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._led_driver_api.static.*))); _led_driver_api_list_end = .;; }
led_strip_driver_api_area : { _led_strip_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._led_strip_driver_api.static.*))); _led_strip_driver_api_list_end = .;; }
lora_driver_api_area : { _lora_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._lora_driver_api.static.*))); _lora_driver_api_list_end = .;; }
mbox_driver_api_area : { _mbox_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._mbox_driver_api.static.*))); _mbox_driver_api_list_end = .;; }
mdio_driver_api_area : { _mdio_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._mdio_driver_api.static.*))); _mdio_driver_api_list_end = .;; }
mipi_dbi_driver_api_area : { _mipi_dbi_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._mipi_dbi_driver_api.static.*))); _mipi_dbi_driver_api_list_end = .;; }
mipi_dsi_driver_api_area : { _mipi_dsi_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._mipi_dsi_driver_api.static.*))); _mipi_dsi_driver_api_list_end = .;; }
mspi_driver_api_area : { _mspi_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._mspi_driver_api.static.*))); _mspi_driver_api_list_end = .;; }
opamp_driver_api_area : { _opamp_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._opamp_driver_api.static.*))); _opamp_driver_api_list_end = .;; }
peci_driver_api_area : { _peci_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._peci_driver_api.static.*))); _peci_driver_api_list_end = .;; }
ps2_driver_api_area : { _ps2_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._ps2_driver_api.static.*))); _ps2_driver_api_list_end = .;; }
ptp_clock_driver_api_area : { _ptp_clock_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._ptp_clock_driver_api.static.*))); _ptp_clock_driver_api_list_end = .;; }
pwm_driver_api_area : { _pwm_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._pwm_driver_api.static.*))); _pwm_driver_api_list_end = .;; }
regulator_parent_driver_api_area : { _regulator_parent_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._regulator_parent_driver_api.static.*))); _regulator_parent_driver_api_list_end = .;; }
regulator_driver_api_area : { _regulator_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._regulator_driver_api.static.*))); _regulator_driver_api_list_end = .;; }
reset_driver_api_area : { _reset_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._reset_driver_api.static.*))); _reset_driver_api_list_end = .;; }
retained_mem_driver_api_area : { _retained_mem_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._retained_mem_driver_api.static.*))); _retained_mem_driver_api_list_end = .;; }
rtc_driver_api_area : { _rtc_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._rtc_driver_api.static.*))); _rtc_driver_api_list_end = .;; }
sdhc_driver_api_area : { _sdhc_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._sdhc_driver_api.static.*))); _sdhc_driver_api_list_end = .;; }
smbus_driver_api_area : { _smbus_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._smbus_driver_api.static.*))); _smbus_driver_api_list_end = .;; }
spi_driver_api_area : { _spi_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._spi_driver_api.static.*))); _spi_driver_api_list_end = .;; }
stepper_driver_api_area : { _stepper_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._stepper_driver_api.static.*))); _stepper_driver_api_list_end = .;; }
syscon_driver_api_area : { _syscon_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._syscon_driver_api.static.*))); _syscon_driver_api_list_end = .;; }
tee_driver_api_area : { _tee_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._tee_driver_api.static.*))); _tee_driver_api_list_end = .;; }
video_driver_api_area : { _video_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._video_driver_api.static.*))); _video_driver_api_list_end = .;; }
virtio_driver_api_area : { _virtio_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._virtio_driver_api.static.*))); _virtio_driver_api_list_end = .;; }
w1_driver_api_area : { _w1_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._w1_driver_api.static.*))); _w1_driver_api_list_end = .;; }
wdt_driver_api_area : { _wdt_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._wdt_driver_api.static.*))); _wdt_driver_api_list_end = .;; }
can_transceiver_driver_api_area : { _can_transceiver_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._can_transceiver_driver_api.static.*))); _can_transceiver_driver_api_list_end = .;; }
nrf_clock_control_driver_api_area : { _nrf_clock_control_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._nrf_clock_control_driver_api.static.*))); _nrf_clock_control_driver_api_list_end = .;; }
i3c_target_driver_api_area : { _i3c_target_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._i3c_target_driver_api.static.*))); _i3c_target_driver_api_list_end = .;; }
its_driver_api_area : { _its_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._its_driver_api.static.*))); _its_driver_api_list_end = .;; }
vtd_driver_api_area : { _vtd_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._vtd_driver_api.static.*))); _vtd_driver_api_list_end = .;; }
renesas_elc_driver_api_area : { _renesas_elc_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._renesas_elc_driver_api.static.*))); _renesas_elc_driver_api_list_end = .;; }
tgpio_driver_api_area : { _tgpio_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._tgpio_driver_api.static.*))); _tgpio_driver_api_list_end = .;; }
pcie_ctrl_driver_api_area : { _pcie_ctrl_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._pcie_ctrl_driver_api.static.*))); _pcie_ctrl_driver_api_list_end = .;; }
pcie_ep_driver_api_area : { _pcie_ep_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._pcie_ep_driver_api.static.*))); _pcie_ep_driver_api_list_end = .;; }
psi5_driver_api_area : { _psi5_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._psi5_driver_api.static.*))); _psi5_driver_api_list_end = .;; }
sent_driver_api_area : { _sent_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._sent_driver_api.static.*))); _sent_driver_api_list_end = .;; }
svc_driver_api_area : { _svc_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._svc_driver_api.static.*))); _svc_driver_api_list_end = .;; }
uart_driver_api_area : { _uart_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._uart_driver_api.static.*))); _uart_driver_api_list_end = .;; }
bc12_emul_driver_api_area : { _bc12_emul_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._bc12_emul_driver_api.static.*))); _bc12_emul_driver_api_list_end = .;; }
bc12_driver_api_area : { _bc12_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._bc12_driver_api.static.*))); _bc12_driver_api_list_end = .;; }
usbc_ppc_driver_api_area : { _usbc_ppc_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._usbc_ppc_driver_api.static.*))); _usbc_ppc_driver_api_list_end = .;; }
tcpc_driver_api_area : { _tcpc_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._tcpc_driver_api.static.*))); _tcpc_driver_api_list_end = .;; }
usbc_vbus_driver_api_area : { _usbc_vbus_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._usbc_vbus_driver_api.static.*))); _usbc_vbus_driver_api_list_end = .;; }
ivshmem_driver_api_area : { _ivshmem_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._ivshmem_driver_api.static.*))); _ivshmem_driver_api_list_end = .;; }
ethphy_driver_api_area : { _ethphy_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._ethphy_driver_api.static.*))); _ethphy_driver_api_list_end = .;; }
ztest :
{
 _ztest_expected_result_entry_list_start = .; KEEP(*(SORT_BY_NAME(._ztest_expected_result_entry.static.*))); _ztest_expected_result_entry_list_end = .;;
 _ztest_suite_node_list_start = .; KEEP(*(SORT_BY_NAME(._ztest_suite_node.static.*))); _ztest_suite_node_list_end = .;;
 _ztest_unit_test_list_start = .; KEEP(*(SORT_BY_NAME(._ztest_unit_test.static.*))); _ztest_unit_test_list_end = .;;
 _ztest_test_rule_list_start = .; KEEP(*(SORT_BY_NAME(._ztest_test_rule.static.*))); _ztest_test_rule_list_end = .;;
}
 init_array :
 {
  __zephyr_init_array_start = .;
  KEEP (*(SORT_BY_INIT_PRIORITY(.init_array.*)
   SORT_BY_INIT_PRIORITY(.ctors.*)))
  KEEP (*(.init_array .ctors))
  __zephyr_init_array_end = .;
  __CTOR_LIST__ = .;
  LONG(0)
  LONG(0)
  __CTOR_END__ = .;
  __init_array_start = .;
  __init_array_end = .;
 }
 bt_l2cap_fixed_chan_area : { _bt_l2cap_fixed_chan_list_start = .; KEEP(*(SORT_BY_NAME(._bt_l2cap_fixed_chan.static.*))); _bt_l2cap_fixed_chan_list_end = .;; }
 bt_gatt_service_static_area : { _bt_gatt_service_static_list_start = .; KEEP(*(SORT_BY_NAME(._bt_gatt_service_static.static.*))); _bt_gatt_service_static_list_end = .;; }
 log_strings_area : { _log_strings_list_start = .; KEEP(*(SORT_BY_NAME(._log_strings.static.*))); _log_strings_list_end = .;; }
 log_stmesp_ptr_area : { _log_stmesp_ptr_list_start = .; KEEP(*(SORT_BY_NAME(._log_stmesp_ptr.static.*))); _log_stmesp_ptr_list_end = .;; }
 log_stmesp_str_area : { _log_stmesp_str_list_start = .; KEEP(*(SORT_BY_NAME(._log_stmesp_str.static.*))); _log_stmesp_str_list_end = .;; }
 log_const_area : { _log_const_list_start = .; KEEP(*(SORT_BY_NAME(._log_const.static.*))); _log_const_list_end = .;; }
 log_backend_area : { _log_backend_list_start = .; KEEP(*(SORT_BY_NAME(._log_backend.static.*))); _log_backend_list_end = .;; }
 log_link_area : { _log_link_list_start = .; KEEP(*(SORT_BY_NAME(._log_link.static.*))); _log_link_list_end = .;; }
 tracing_backend_area : { _tracing_backend_list_start = .; KEEP(*(SORT_BY_NAME(._tracing_backend.static.*))); _tracing_backend_list_end = .;; }
 zephyr_dbg_info :
 {
  KEEP(*(".dbg_thread_info"));
 }
 intc_table_area : { _intc_table_list_start = .; KEEP(*(SORT_BY_NAME(._intc_table.static.*))); _intc_table_list_end = .;; }
 input_callback_area : { _input_callback_list_start = .; KEEP(*(SORT_BY_NAME(._input_callback.static.*))); _input_callback_list_end = .;; }
 symbol_to_keep :
 {
  __symbol_to_keep_start = .;
  KEEP(*(SORT(.symbol_to_keep*)));
  __symbol_to_keep_end = .;
 }
 shell_area : { _shell_list_start = .; KEEP(*(SORT_BY_NAME(._shell.static.*))); _shell_list_end = .;; }
 shell_root_cmds_area : { _shell_root_cmds_list_start = .; KEEP(*(SORT_BY_NAME(._shell_root_cmds.static.*))); _shell_root_cmds_list_end = .;; }
 shell_subcmds_area : { _shell_subcmds_list_start = .; KEEP(*(SORT_BY_NAME(._shell_subcmds.static.*))); _shell_subcmds_list_end = .;; }
 shell_dynamic_subcmds_area : { _shell_dynamic_subcmds_list_start = .; KEEP(*(SORT_BY_NAME(._shell_dynamic_subcmds.static.*))); _shell_dynamic_subcmds_list_end = .;; }
 cfb_font_area : { _cfb_font_list_start = .; KEEP(*(SORT_BY_NAME(._cfb_font.static.*))); _cfb_font_list_end = .;; }
rodata :
{
}
datas :
{
}
        device_states :
        {
  . = ALIGN(4);
                __device_states_start = .;
  KEEP(*(".z_devstate"));
  KEEP(*(".z_devstate.*"));
                __device_states_end = .;
  . = ALIGN(4);
        }
 log_mpsc_pbuf_area : { _log_mpsc_pbuf_list_start = .; *(SORT_BY_NAME(._log_mpsc_pbuf.static.*)); _log_mpsc_pbuf_list_end = .;; }
 log_msg_ptr_area : { _log_msg_ptr_list_start = .; KEEP(*(SORT_BY_NAME(._log_msg_ptr.static.*))); _log_msg_ptr_list_end = .;; }
 log_dynamic_area : { _log_dynamic_list_start = .; KEEP(*(SORT_BY_NAME(._log_dynamic.static.*))); _log_dynamic_list_end = .;; }
 k_timer_area : { _k_timer_list_start = .; *(SORT_BY_NAME(._k_timer.static.*)); _k_timer_list_end = .;; }
 k_mem_slab_area : { _k_mem_slab_list_start = .; *(SORT_BY_NAME(._k_mem_slab.static.*)); _k_mem_slab_list_end = .;; }
 k_heap_area : { _k_heap_list_start = .; *(SORT_BY_NAME(._k_heap.static.*)); _k_heap_list_end = .;; }
 k_mutex_area : { _k_mutex_list_start = .; *(SORT_BY_NAME(._k_mutex.static.*)); _k_mutex_list_end = .;; }
 k_stack_area : { _k_stack_list_start = .; *(SORT_BY_NAME(._k_stack.static.*)); _k_stack_list_end = .;; }
 k_msgq_area : { _k_msgq_list_start = .; *(SORT_BY_NAME(._k_msgq.static.*)); _k_msgq_list_end = .;; }
 k_mbox_area : { _k_mbox_list_start = .; *(SORT_BY_NAME(._k_mbox.static.*)); _k_mbox_list_end = .;; }
 k_pipe_area : { _k_pipe_list_start = .; *(SORT_BY_NAME(._k_pipe.static.*)); _k_pipe_list_end = .;; }
 k_sem_area : { _k_sem_list_start = .; *(SORT_BY_NAME(._k_sem.static.*)); _k_sem_list_end = .;; }
 k_event_area : { _k_event_list_start = .; *(SORT_BY_NAME(._k_event.static.*)); _k_event_list_end = .;; }
 k_queue_area : { _k_queue_list_start = .; *(SORT_BY_NAME(._k_queue.static.*)); _k_queue_list_end = .;; }
 k_fifo_area : { _k_fifo_list_start = .; *(SORT_BY_NAME(._k_fifo.static.*)); _k_fifo_list_end = .;; }
 k_lifo_area : { _k_lifo_list_start = .; *(SORT_BY_NAME(._k_lifo.static.*)); _k_lifo_list_end = .;; }
 k_condvar_area : { _k_condvar_list_start = .; *(SORT_BY_NAME(._k_condvar.static.*)); _k_condvar_list_end = .;; }
 sys_mem_blocks_ptr_area : { _sys_mem_blocks_ptr_list_start = .; *(SORT_BY_NAME(._sys_mem_blocks_ptr.static.*)); _sys_mem_blocks_ptr_list_end = .;; }
 net_buf_pool_area : { _net_buf_pool_list_start = .; KEEP(*(SORT_BY_NAME(._net_buf_pool.static.*))); _net_buf_pool_list_end = .;; }
native_pre_tasks :
{
 __native_tasks_start = .;
 __native_PRE_BOOT_1_tasks_start = .; KEEP(*(SORT(.native_PRE_BOOT_1[0-9]_task))); KEEP(*(SORT(.native_PRE_BOOT_1[1-9][0-9]_task))); KEEP(*(SORT(.native_PRE_BOOT_1[1-9][0-9][0-9]_task)));
 __native_PRE_BOOT_2_tasks_start = .; KEEP(*(SORT(.native_PRE_BOOT_2[0-9]_task))); KEEP(*(SORT(.native_PRE_BOOT_2[1-9][0-9]_task))); KEEP(*(SORT(.native_PRE_BOOT_2[1-9][0-9][0-9]_task)));
 __native_PRE_BOOT_3_tasks_start = .; KEEP(*(SORT(.native_PRE_BOOT_3[0-9]_task))); KEEP(*(SORT(.native_PRE_BOOT_3[1-9][0-9]_task))); KEEP(*(SORT(.native_PRE_BOOT_3[1-9][0-9][0-9]_task)));
 __native_FIRST_SLEEP_tasks_start = .; KEEP(*(SORT(.native_FIRST_SLEEP[0-9]_task))); KEEP(*(SORT(.native_FIRST_SLEEP[1-9][0-9]_task))); KEEP(*(SORT(.native_FIRST_SLEEP[1-9][0-9][0-9]_task)));
 __native_ON_EXIT_tasks_start = .; KEEP(*(SORT(.native_ON_EXIT[0-9]_task))); KEEP(*(SORT(.native_ON_EXIT[1-9][0-9]_task))); KEEP(*(SORT(.native_ON_EXIT[1-9][0-9][0-9]_task)));
 __native_tasks_end = .;
}
__data_region_end = .;
noinit :
{
}
} INSERT AFTER .data;
SECTIONS
{
 .native_sim_if.data :
 {
  KEEP(*(.native_sim_if.data));
 }
} INSERT AFTER .data;
SECTIONS
{
 .native_sim_if :
 {
  KEEP(*(.native_sim_if));
  KEEP(*(.native_sim_if.*));
 }
} INSERT AFTER .text;
