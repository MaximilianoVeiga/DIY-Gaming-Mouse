# Hardware verification

Use this checklist after flashing improved firmware. CI only proves compilation with the dummy SROM path.

## Boot / sensor

- [ ] Serial shows `product id: 66 (inverse: 189)` (0x42 / 0xBD)
- [ ] With real SROM: non-zero `srom id` and `observation` with bit 6 set (`ok`)
- [ ] Without real SROM: degraded-mode warning, buttons/wheel/motion still function

## Settings persistence

- [ ] DPI chord (LMB+RMB+DPI+M4 + wheel) changes DPI; after ~2 s serial prints `settings: saved`
- [ ] Power cycle restores DPI (`settings: loaded`)
- [ ] LOD chord (LMB+RMB+DPI+M5 + wheel) changes 2–3 mm and persists the same way
- [ ] Corrupt/missing KV record falls back to 1200 DPI / 2 mm LOD

## USB / input

- [ ] Rapid motion + clicks do not leave buttons stuck
- [ ] Under host stress, occasional `usb: recovered` is acceptable; cursor should not jump wildly after recover
- [ ] Wheel does not skip at moderate scroll speeds (~1 kHz report path)

## Limp / recovery

- [ ] Disconnecting sensor SPI (or bad wiring) enters limp mode: motion stops, buttons/wheel still work
- [ ] After reconnect and a few idle seconds, recovery attempt restores motion when IDs validate
