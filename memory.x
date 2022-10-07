MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */
  /* These values correspond to the NRF52840 with Softdevices S140 7.0.1 */
  FLASH : ORIGIN = 0x00027000, LENGTH = 868K /* (1024K - 156K) */ 
  RAM : ORIGIN = 0x20020000, LENGTH = 128K /* (256K - 128K) */
}
