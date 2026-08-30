# Zigbee Tester ESP32-C6 — TODO

## Infrastructură
- [x] State machine UI (MAIN, GENERATOR, SNIFFER, SETUP, INFO)
- [x] Structuri radio separate (stats, settings, frame buffer)
- [x] Parser IEEE 802.15.4 de bază
- [x] Driver touchscreen XPT2046/HR2046 placeholder
- [x] Funcție TX test frame

## UI / Ecrane
- [x] Meniu principal cu butoane touchscreen
- [x] Generator LOCKED (START/STOP, afișare contoare)
- [x] Generator SPREAD (matrice canale selectabile, ALL/NONE, START/STOP)
- [x] Sniffer LOCKED (canal fix, RSSI, LQI, frame info)
- [x] Sniffer SCAN (channel hopping 11-26, bar chart)
- [x] LAST FRAME (payload hex, tip, adrese)
- [x] INFO (diagnostic minimal)
- [ ] SETUP — implementare editare parametrii (PAN ID, channel, TX interval, dwell)
- [ ] Paginare / scroll în LAST FRAME pentru payload lung
- [ ] Touch calibration și mapare precisă pe 128x160

## Hardware
- [ ] Setare reală pini touch (T_IRQ, T_DO, T_DIN, T_CS, T_CLK)
- [ ] Testare touch pe ecran real
- [ ] Ajustare calibrare touch (TOUCH_X_MIN/MAX, Y_MIN/MAX)

## Radio
- [ ] Test TX în Generator LOCKED
- [ ] Test TX în Generator SPREAD cu channel hopping
- [ ] Verificare parsing frame recepționat
- [ ] Verificare scan RSSI/pachete pe canale
- [ ] Salvare setări în NVS

## Polish
- [ ] Eliminare warnings funcții/variabile nefolosite
- [ ] Refactor în fișiere separate (tft.c, touch.c, radio.c, ui.c)
- [ ] Documentare pin mapping final
