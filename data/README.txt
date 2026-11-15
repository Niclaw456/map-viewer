This 'data' folder is a placeholder. The map viewer reads tiles from the
MicroSD card, NOT from flash, so nothing here is uploaded to the device.

All app data lives under one root folder on the card, /offgridmap. Put your
generated tiles there:
    SD:/offgridmap/tiles/{z}/{x}/{y}.png      (OSM Standard)
    SD:/offgridmap/topo/{z}/{x}/{y}.png       (OpenTopoMap)
    SD:/offgridmap/cyclosm/{z}/{x}/{y}.png    (CyclOSM)

The firmware creates /offgridmap itself at boot if it's missing, and writes
settings.txt, waypoints.csv, wifi.txt and the tracks/ folder inside it too.
