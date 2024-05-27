# Sound Mixer

## Introducere

_Where words fail, music speaks_ spunea Hans Christian Andersen. Muzica e cu noi oriunde, ne dă puterea de a ne exprima într-un mod unic, ne unește și ne face să ne simțim mai bine. Dar, mai presus de toate, muzica e despre creativitate. Iar de multe ori creativitatea înseamnă să vii cu propria reinterpretare asupra muncii altor oameni.

Astfel, m-am gândit să construiesc un dispozitiv care să ofere oricui și oriunde posibilitatea de a remixa melodii. Fără echipament costisitor și dificil de transportat ca cel de DJ, oricând te lovește inspirația poți începe să creezi fără întârziere! 

## Despre implementare, pe scurt

Mediu de dezvoltare: PlatformIO, Visual Studio Code

Biblioteci utilizate:
- Petit FatFs - FAT file system library: pentru citirea melodiilor de pe cardul microSD
- C Library for SSD1306 0.96” OLED display: pentru afișarea informațiilor pe display

Detalii implementare:
- Citirea butoanelor se face folosind întreruperi de tip pin-change interrupt și rezistențele de pull-up interne.
- Pentru citirea potențiometrului liniar am folosit funcția implementată în laboratorul de ADC (myAnalogRead).
- Pentru citirea de pe cardul microSD am folosit scheletul și implementarea din laboratorul de SPI, pe care l-am modificat pentru a adăuga funcționalități noi / a oferi suport pentru cazuri netratate.
- Pentru afișarea pe display prin I2C am folosit funcțiile din biblioteca menționată mai sus.
- Redarea melodiei, afișarea timpului parcurs în redarea acesteia și debounce-ul butoanelor se realizează folosind timere.
- Redarea melodiei se realizează prin output PWM al Timer1.
